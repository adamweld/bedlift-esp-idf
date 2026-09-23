#ifndef BEDLIFT_UI_PANELS_HPP
#define BEDLIFT_UI_PANELS_HPP

// Production UI panels. Draw-only: everything renders into a caller-owned
// full-frame LGFX_Sprite from a sys_snapshot_t — no tasks, no state, no
// platform calls — so the identical pixels appear in hostsim and on the
// ST7789. The render task (target) / host loop owns pacing and pushing.

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "sys_state.h"

// Layout (240x135, matches the experiments/display geometry)
#define UI_W 240
#define UI_H 135
#define UI_BTN_W 46
#define UI_MODE_W 66

// The 1.14" ST7789 panel's pixels aren't square: active area ~24.91 x 14.86 mm
// over 240 x 135 px = 0.1038 mm wide x 0.1101 mm tall. Anything that must
// look round on the glass (the bubble level) is drawn this much wider than
// tall in pixels. Hostsim renders square pixels, so there it looks oval.
#define UI_PX_ASPECT (0.1101f / 0.1038f)   // pixel height / width

// Full-frame composition: mode panel (with status icons), level display (or
// fault detail when latched), button panel, warning banner.
void ui_render(LGFX_Sprite &fb, const sys_snapshot_t &s);

#endif
