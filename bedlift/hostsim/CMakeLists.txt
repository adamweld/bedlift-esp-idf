cmake_minimum_required(VERSION 3.16)

# xcrun defaults to the highest-numbered SDK (e.g. 27.0), which can be newer
# than the installed CLT linker supports. The MacOSX.sdk symlink tracks the
# CLT's canonical SDK, which always matches its linker — pin that unless the
# caller overrides SDKROOT.
if(APPLE AND NOT DEFINED ENV{SDKROOT})
    set(CMAKE_OSX_SYSROOT /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk)
endif()

project(bedlift_hostsim CXX C)

add_definitions(-DLGFX_SDL)

set(LGFX ${CMAKE_CURRENT_SOURCE_DIR}/../components/LovyanGFX)

file(GLOB LGFX_SRC CONFIGURE_DEPENDS
    ${LGFX}/src/lgfx/Fonts/efont/*.c
    ${LGFX}/src/lgfx/Fonts/IPA/*.c
    ${LGFX}/src/lgfx/Fonts/lvgl/*.c
    ${LGFX}/src/lgfx/utility/*.c
    ${LGFX}/src/lgfx/v1/*.cpp
    ${LGFX}/src/lgfx/v1/lv_font/*.c
)

add_executable(hostsim
    main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/cybergear/cybergear.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/cybergear/cybergear_selftest.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/sim/sim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/buttons/buttons_sm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/ui/ui_panels.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/motion/motion_fsm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/motion/control_law.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/safety/hazard_checks.c
    ${LGFX_SRC}
)

target_include_directories(hostsim PRIVATE
    ${LGFX}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/cybergear/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/sim/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/buttons/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/state/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/ui/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/ui/generated
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/motion/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/safety/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../main
)

find_package(SDL2 REQUIRED COMPONENTS SDL2)
target_link_libraries(hostsim PRIVATE SDL2::SDL2)
target_compile_features(hostsim PUBLIC cxx_std_17)
