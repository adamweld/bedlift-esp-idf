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
#define UI_STATUS_H 26
#define UI_BTN_W 46
#define UI_MODE_W 66

// Full-frame composition: status bar, mode panel, level display (or fault
// detail when latched), button panel.
void ui_render(LGFX_Sprite &fb, const sys_snapshot_t &s);

#endif
