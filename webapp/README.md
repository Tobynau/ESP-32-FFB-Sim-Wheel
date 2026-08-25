# FFB Wheel Config (web app)

Browser-based tuning/calibration tool for the wheel. Single static page, no
build step, no drivers, no installed software — it talks to the wheel with
**WebHID** over the vendor feature reports described in
[CONFIG_PROTOCOL.md](../CONFIG_PROTOCOL.md).

Hosted by GitHub Pages at `https://tobynau.github.io/ESP-32-FFB-Sim-Wheel/`
(deployed from `webapp/public/` by `.github/workflows/static.yml`).

## Use

1. Plug the wheel in and open the page in **Chrome or Edge** (WebHID isn't in
   Firefox/Safari). Local testing works from `file://` or any local server.
2. **Connect** → pick "ESP32-S3 FFB Wheel" in the chooser.
3. Tune while driving — **Apply (RAM)** sends without persisting, **Apply &
   Save** writes to NVS.
4. **Calibrate** each pedal: click, move it through full travel (brake: press
   to your desired 100%), click **Done**, then Apply & Save.
5. **Set center** with the wheel held straight.

**Eject before gaming.** While this page holds the wheel open, games can't
acquire it (the browser opens the whole HID device, not just the vendor
collection). Click **Eject**, or just switch tabs — the page releases the
wheel automatically when hidden or closed.
