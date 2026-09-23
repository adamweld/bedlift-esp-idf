idf_component_register(SRCS "ui_panels.cpp" "ui_icons.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES LovyanGFX state)

# icons/*.png -> ui_icons_data.cpp (see icons/icons.txt)
include(${COMPONENT_DIR}/icons.cmake)
bedlift_ui_icons(${COMPONENT_LIB})
