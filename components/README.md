# components/

Libraries the ESP-IDF build needs that the component manager cannot fetch.

Everything else comes from `main/idf_component.yml` — arduino-esp32, m5unified
(which pulls m5gfx), usb_host_msc — and lands in `managed_components/`.

`sunset` is not on the registry: it is a plain Arduino library with no
`idf_component.yml`. Clone it once:

```sh
cd components/sunset
git clone --depth 1 https://github.com/buelowp/sunset.git
```

Note that `main/CMakeLists.txt` names `sunset` in its `REQUIRES`. That is not
optional: `main` only requires every component automatically while its
`REQUIRES` is unset, and this project sets it.

The Arduino build ignores this directory — `arduino-cli` compiles the sketch
folder and `src/` only.
