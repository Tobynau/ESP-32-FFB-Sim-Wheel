const controls = [
  'max_angle_deg',
  'angle_limit_stiffness',
  'overall_strength',
  'motor_max_amps',
  'device_gain',
  'scale_constant',
  'scale_spring',
  'scale_damper',
  'scale_inertia',
  'scale_friction',
  'scale_periodic',
];

let port;
let reader;
let writer;
let readLoopRunning = false;
let encoder;
let decoder;
const pendingSetTimers = new Map();

const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connectBtn');
const disconnectBtn = document.getElementById('disconnectBtn');
const recenterBtn = document.getElementById('recenterBtn');

function setStatus(text) {
  statusEl.textContent = text;
}

function setConnectedUi(connected) {
  connectBtn.disabled = connected;
  disconnectBtn.disabled = !connected;
  recenterBtn.disabled = !connected;
}

function updateOutput(id, value) {
  const out = document.getElementById(`${id}_out`);
  if (!out) return;
  out.textContent = Number(value).toFixed(2);
}

function applyConfig(cfg) {
  controls.forEach((id) => {
    if (cfg[id] == null) return;
    const input = document.getElementById(id);
    if (!input) return;
    input.value = cfg[id];
    updateOutput(id, cfg[id]);
  });
}

function applyTelemetry(tel) {
  document.getElementById('tel_angle').textContent = `${(tel.angle_deg ?? 0).toFixed(2)}°`;
  document.getElementById('tel_vel').textContent = `${(tel.vel_rads ?? 0).toFixed(2)} rad/s`;
  document.getElementById('tel_buttons').textContent = `${tel.buttons ?? 0}`;
  document.getElementById('tel_left').textContent = `${tel.left_paddle ?? 0}`;
  document.getElementById('tel_right').textContent = `${tel.right_paddle ?? 0}`;
  document.getElementById('tel_cmd').textContent = `${(tel.current_cmd_a ?? 0).toFixed(2)} A`;
  document.getElementById('tel_motor').textContent = `${(tel.current_motor_a ?? 0).toFixed(2)} A`;
  document.getElementById('tel_ffb').textContent = `${tel.ffb_active ?? 0}`;
}

async function sendLine(line) {
  if (!writer) return;
  const payload = `${line}\n`;
  await writer.write(encoder.encode(payload));
}

async function requestConfig() {
  await sendLine('get');
}

async function connectSerial() {
  if (!('serial' in navigator)) {
    setStatus('Web Serial unsupported in this browser');
    return;
  }

  port = await navigator.serial.requestPort();
  await port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: 'none', flowControl: 'none' });

  encoder = new TextEncoder();
  decoder = new TextDecoder();
  writer = port.writable.getWriter();
  readLoopRunning = true;
  setConnectedUi(true);
  setStatus('Connected');

  startReadLoop();
  await requestConfig();
}

async function disconnectSerial() {
  readLoopRunning = false;

  if (reader) {
    try { await reader.cancel(); } catch (_) {}
    try { reader.releaseLock(); } catch (_) {}
    reader = null;
  }

  if (writer) {
    try { writer.releaseLock(); } catch (_) {}
    writer = null;
  }

  if (port) {
    try { await port.close(); } catch (_) {}
    port = null;
  }

  setConnectedUi(false);
  setStatus('Disconnected');
}

async function startReadLoop() {
  let buffer = '';
  while (port && port.readable && readLoopRunning) {
    reader = port.readable.getReader();
    try {
      while (readLoopRunning) {
        const { value, done } = await reader.read();
        if (done) break;
        buffer += decoder.decode(value, { stream: true });
        let idx;
        while ((idx = buffer.indexOf('\n')) >= 0) {
          const line = buffer.slice(0, idx).trim();
          buffer = buffer.slice(idx + 1);
          if (!line) continue;
          try {
            const msg = JSON.parse(line);
            if (msg.type === 'config') applyConfig(msg);
            if (msg.type === 'telemetry') applyTelemetry(msg);
          } catch (_) {
            // ignore non-JSON lines
          }
        }
      }
    } catch (_) {
      // ignore; disconnect handles state
    } finally {
      reader.releaseLock();
      reader = null;
    }
  }
}

function scheduleSet(id, value) {
  if (!port || !writer) return;
  if (pendingSetTimers.has(id)) {
    clearTimeout(pendingSetTimers.get(id));
  }
  const timer = setTimeout(() => {
    sendLine(`set ${id} ${value}`).catch(() => {});
    pendingSetTimers.delete(id);
  }, 80);
  pendingSetTimers.set(id, timer);
}

controls.forEach((id) => {
  const input = document.getElementById(id);
  input.addEventListener('input', () => {
    const value = Number(input.value);
    updateOutput(id, value);
    scheduleSet(id, value);
  });
});

connectBtn.addEventListener('click', async () => {
  try {
    await connectSerial();
  } catch (err) {
    setStatus(`Connect failed: ${err.message}`);
    await disconnectSerial();
  }
});

disconnectBtn.addEventListener('click', async () => {
  await disconnectSerial();
});

recenterBtn.addEventListener('click', async () => {
  try {
    await sendLine('recenter');
  } catch (_) {}
});

window.addEventListener('beforeunload', () => {
  disconnectSerial();
});

setConnectedUi(false);
controls.forEach((id) => {
  const input = document.getElementById(id);
  updateOutput(id, Number(input.value));
});
