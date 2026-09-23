# Icon generation, shared by the ESP-IDF component and the host simulator.
#
#   include(<ui>/icons.cmake)
#   bedlift_ui_icons(<target>)
#
# Runs scripts/gen_icons.py (via uv) whenever assets/icons/icons.txt, an icon
# or the script changes, and adds the generated ui_icons_data.cpp to <target>.

set(BEDLIFT_UI_DIR ${CMAKE_CURRENT_LIST_DIR})

function(bedlift_ui_icons target)
    find_program(BEDLIFT_UV uv HINTS $ENV{HOME}/.local/bin /opt/homebrew/bin)
    if(NOT BEDLIFT_UV)
        message(FATAL_ERROR "bedlift ui: 'uv' not found; it runs "
                "components/ui/scripts/gen_icons.py to build the icon table. "
                "Install uv (https://docs.astral.sh/uv/) or put it on PATH.")
    endif()

    set(icons_dir ${BEDLIFT_UI_DIR}/../../assets/icons)
    get_filename_component(icons_dir ${icons_dir} ABSOLUTE)
    set(script ${BEDLIFT_UI_DIR}/scripts/gen_icons.py)
    set(out ${CMAKE_CURRENT_BINARY_DIR}/ui_icons_data.cpp)
    file(GLOB sources CONFIGURE_DEPENDS ${icons_dir}/*.png ${icons_dir}/*.svg)

    add_custom_command(
        OUTPUT ${out}
        COMMAND ${BEDLIFT_UV} run --quiet --script ${script} ${icons_dir} ${out}
        DEPENDS ${script} ${icons_dir}/icons.txt ${sources}
        COMMENT "Generating UI icons"
        VERBATIM)
    target_sources(${target} PRIVATE ${out})
endfunction()
