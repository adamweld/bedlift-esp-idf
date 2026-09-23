#include <cstring>
#include "ui_icons.hpp"

const ui_icon_t *ui_icon(const char *name)
{
    for (int i = 0; i < ui_icon_count; i++)
        if (strcmp(ui_icon_table[i].name, name) == 0) return &ui_icon_table[i];
    return nullptr;
}

void ui_icon_draw(LGFX_Sprite &fb, const char *name, int x, int y,
                  uint16_t fg, int rot)
{
    const ui_icon_t *ic = ui_icon(name);
    if (!ic) {
        fb.drawRect(x, y, 16, 16, fb.color565(255, 0, 255));
        fb.drawLine(x, y, x + 15, y + 15, fb.color565(255, 0, 255));
        return;
    }
    rot &= 3;
    const int w = ic->w, h = ic->h;
    const int dw = (rot & 1) ? h : w, dh = (rot & 1) ? w : h;
    // fg is RGB565; expand once to 8-bit channels for blending
    const int fr = ((fg >> 11) & 0x1F) * 255 / 31;
    const int fgc = ((fg >> 5) & 0x3F) * 255 / 63;
    const int fb_ = (fg & 0x1F) * 255 / 31;

    for (int j = 0; j < dh; j++) {
        int py = y + j;
        if (py < 0 || py >= fb.height()) continue;
        for (int i = 0; i < dw; i++) {
            int px = x + i;
            if (px < 0 || px >= fb.width()) continue;
            int sx, sy;                       // dest (i,j) -> source pixel
            switch (rot) {
                case 1:  sx = j;         sy = h - 1 - i; break;
                case 2:  sx = w - 1 - i; sy = h - 1 - j; break;
                case 3:  sx = w - 1 - j; sy = i;         break;
                default: sx = i;         sy = j;         break;
            }
            int k = ic->a8[sy * w + sx];
            if (!k) continue;
            if (k == 255) { fb.drawPixel(px, py, fg); continue; }
            auto bg = fb.readPixelRGB(px, py);
            fb.drawPixel(px, py, fb.color888((fr * k + bg.r * (255 - k)) / 255,
                                             (fgc * k + bg.g * (255 - k)) / 255,
                                             (fb_ * k + bg.b * (255 - k)) / 255));
        }
    }
}

void ui_icon_draw_centered(LGFX_Sprite &fb, const char *name, int cx, int cy,
                           uint16_t fg, int rot)
{
    const ui_icon_t *ic = ui_icon(name);
    int w = ic ? ic->w : 16, h = ic ? ic->h : 16;
    if (rot & 1) { int t = w; w = h; h = t; }
    ui_icon_draw(fb, name, cx - w / 2, cy - h / 2, fg, rot);
}
