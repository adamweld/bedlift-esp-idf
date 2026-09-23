#ifndef BEDLIFT_UI_ICONS_HPP
#define BEDLIFT_UI_ICONS_HPP

// Named UI icons. Source PNGs live in components/ui/icons/, listed with their
// target size in icons/icons.txt; the build converts them (scripts/
// gen_icons.py) into 8-bit coverage maps, looked up here by name.

#include <cstdint>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

struct ui_icon_t {
    const char *name;
    uint8_t w, h;
    const uint8_t *a8;       // w*h coverage, row-major, 0 = transparent
};

// generated table
extern const ui_icon_t ui_icon_table[];
extern const int ui_icon_count;

// nullptr when the name isn't in icons.txt
const ui_icon_t *ui_icon(const char *name);

// Draw icon `name` with its top-left at (x, y) in `fg`, anti-aliased over
// whatever is already in the frame. rot = clockwise quarter turns (0..3).
// An unknown name draws a magenta box so it can't go unnoticed.
void ui_icon_draw(LGFX_Sprite &fb, const char *name, int x, int y,
                  uint16_t fg, int rot = 0);

// Same, centered on (cx, cy).
void ui_icon_draw_centered(LGFX_Sprite &fb, const char *name, int cx, int cy,
                           uint16_t fg, int rot = 0);

#endif
