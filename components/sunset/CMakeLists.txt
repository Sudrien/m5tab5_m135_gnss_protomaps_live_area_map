# sunset, as an ESP-IDF component.
#
# A plain Arduino library: no idf_component.yml, so the component manager
# cannot fetch it, and no CMakeLists.txt of its own. Wrapping it takes three
# lines because it turns out not to be Arduino-dependent at all - sunset.h
# includes only <cmath> and <ctime>, and sunset.cpp only its own header. It is
# sun-angle arithmetic that happens to ship as an Arduino library.
#
# No REQUIRES: nothing from IDF or Arduino is needed to build it. Note that
# `main` must name `sunset` in *its* REQUIRES, because main passes REQUIRES
# explicitly and that replaces the otherwise-automatic dependency on every
# component.
#
# Expected layout, with the upstream checkout untouched underneath:
#
#   components/sunset/CMakeLists.txt   <- this file
#   components/sunset/sunset/          <- git clone of buelowp/sunset
#                     └── src/sunset.{h,cpp}
#
#   cd components/sunset
#   git clone --depth 1 https://github.com/buelowp/sunset.git
#
# Keeping the clone in a subdirectory rather than as the component root means
# `git pull` there stays clean - this file is not dropped into someone else's
# repository.
#
# Upstream master is 1.1.7, which is what arduino-cli reports for the Arduino
# build, so both builds get the same library.

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/sunset/src/sunset.cpp)
    # The project's top-level CMakeLists clones this before project(), so
    # reaching here means that step failed or was skipped - most likely no
    # network on the first configure.
    message(FATAL_ERROR
        "sunset sources not found. The top-level CMakeLists should have cloned "
        "them; if it could not reach github, do it by hand:\n"
        "  git clone --depth 1 https://github.com/buelowp/sunset.git "
        "${CMAKE_CURRENT_LIST_DIR}/sunset")
endif()

idf_component_register(
    SRCS         "sunset/src/sunset.cpp"
    INCLUDE_DIRS "sunset/src"
)
