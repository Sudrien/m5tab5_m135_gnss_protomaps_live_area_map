m5tab5_m135_gnss_protomaps_live_area_map

https://docs.m5stack.com/en/core/Tab5

https://docs.m5stack.com/en/module/GNSS%20Module

https://docs.protomaps.com/pmtiles/


A nicer cover for the module:
https://www.printables.com/model/1805013-simple-m5stack-bottom-tab5-m135-gnss



The ESP-IDF build (5.5.5 currently, 6.x not supported by m5Unified as of writing)
> (idf.py set-target esp32p4 || rm -rf build && idf.py set-target esp32p4) && idf.py build flash monitor

the arduino build
> arduino-cli compile --fqbn "esp32:esp32:m5stack_tab5:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc" . && arduino-cli upload --fqbn "esp32:esp32:m5stack_tab5:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc" --port /dev/ttyACM0 . && sleep 2 && arduino-cli monitor --port /dev/ttyACM0 --config baudrate=115200


The arduino build is more streamlined. the ESP-IDF build is trying to support files on usb stick, exfat (not just fat32), and hotplug.



Modes:
- wifi tile download only
-- can only get the immediate area
- world file download
-- prepopulate everything on earth
-- it's 38 gigs downloading peacemeal
-- yes I want to support just offlining the whole file but that requires a larger than 129GB microsd right now. Because formatting a filesystem takes space.

- Supports GNSS caching on microsd to speed up location aquisition
- Remembers multiple wifi networks
- 'rule of thirds' sliding display - this will keep up if you're in a car.
- night mode
- screen toggle for battery savings
