# The sources live one level up, shared verbatim with the Arduino build.
#
# tab5_map.ino is deliberately not listed: it is an empty placeholder that
# exists only because arduino-cli requires a .ino matching the folder name.
# The sketch itself is tab5_map.cpp.

idf_component_register(
    SRCS
        "../tab5_map.cpp"
        "../mapengine.cpp"
        "../netsource.cpp"
        "../tilecache.cpp"
        "../wifistore.cpp"
        "../portal.cpp"
        "../gnss.cpp"
        "../storage.cpp"
        "../raster.c"
        "../mvt.c"
        "../inflate.c"
        "../pmtiles.c"
        "../style.c"
        "../tile_grid.c"
        "idf_usb_msc.cpp"

    INCLUDE_DIRS ".." "."
    ## `main` normally requires every other component automatically - but that
    ## only holds while REQUIRES is left unset. Passing it explicitly, as here,
    ## replaces the automatic list, so anything not named becomes invisible.
    ##
    ## That is what hid sunset: the component registered and compiled its own
    ## source perfectly well, and its include directory simply never reached
    ## main. It shows up in compile_commands.json on sunset's own compile line
    ## and nowhere else, which is a confusing thing to stare at.
    ##
    ## Managed dependencies (arduino-esp32, m5unified, usb_host_msc) are still
    ## absent from this list on purpose: the component manager adds them from
    ## idf_component.yml, and naming arduino here is what produced "Failed to
    ## resolve component 'arduino'" - a managed copy registers as
    ## espressif__arduino-esp32, not arduino.
    REQUIRES
        fatfs
        vfs
        sunset
)

# Tells features.h which build this is. Deliberately explicit rather than
# inferred from ARDUINO, which is defined in both configurations.
target_compile_definitions(${COMPONENT_LIB} PUBLIC MAP_BUILD_IDF=1)
