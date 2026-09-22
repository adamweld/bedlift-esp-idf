#include <string.h>
#include "buttons_sm.h"

static void emit(btn_sm_t *sm, btn_id_e id, btn_event_e type, int64_t t)
{
    int next = (sm->q_head + 1) % 16;
    if (next == sm->q_tail) return;   // overflow: drop oldest-first policy N/A, drop new
    sm->q[sm->q_head] = (btn_event_t){ .id = id, .type = type, .t_us = t };
    sm->q_head = next;
}

void btn_sm_init(btn_sm_t *sm)
{
    memset(sm, 0, sizeof(*sm));
    sm->debounce_us = 20 * 1000;
    sm->chord_us = 80 * 1000;
    sm->hold_us = 600 * 1000;
    sm->repeat_us = 150 * 1000;
}

void btn_sm_init_latched(btn_sm_t *sm)
{
    btn_sm_init(sm);
    sm->wake_latch = true;
}

bool btn_sm_poll(btn_sm_t *sm, btn_event_t *out)
{
    if (sm->q_tail == sm->q_head) return false;
    *out = sm->q[sm->q_tail];
    sm->q_tail = (sm->q_tail + 1) % 16;
    return true;
}

static bool debounced(btn_sm_t *sm, int i, bool raw, int64_t now)
{
    if (raw != sm->b[i].raw) {
        sm->b[i].raw = raw;
        sm->b[i].last_edge_us = now;
    }
    if (raw != sm->b[i].stable &&
        now - sm->b[i].last_edge_us >= sm->debounce_us) {
        sm->b[i].stable = raw;
        return true;   // stable state changed
    }
    return false;
}

void btn_sm_step(btn_sm_t *sm, bool up, bool mode, bool down, int64_t now)
{
    bool raw[BTN_COUNT] = { up, mode, down };
    bool changed[BTN_COUNT];
    for (int i = 0; i < BTN_COUNT; i++)
        changed[i] = debounced(sm, i, raw[i], now);

    // Wake latch: swallow everything until all released
    if (sm->wake_latch) {
        if (!sm->b[BTN_UP].stable && !sm->b[BTN_MODE].stable &&
            !sm->b[BTN_DOWN].stable)
            sm->wake_latch = false;
        return;
    }

    // ---- chord layer (UP/DOWN only) ----------------------------------------
    for (int i = 0; i < BTN_COUNT; i += 2) {           // BTN_UP, BTN_DOWN
        int other = (i == BTN_UP) ? BTN_DOWN : BTN_UP;
        if (changed[i] && sm->b[i].stable) {
            sm->b[i].press_us = now;
            if (sm->b[other].pending || sm->b[other].in_chord ||
                (sm->b[other].stable && sm->chord_active)) {
                // partner already waiting → chord
                sm->b[i].in_chord = true;
                sm->b[other].in_chord = true;
                sm->b[other].pending = false;
                sm->b[i].pending = false;
                if (!sm->chord_active) {
                    sm->chord_active = true;
                    emit(sm, BTN_UP, BEV_CHORD_UPDOWN, now);
                }
            } else if (sm->b[other].stable && sm->b[other].reported) {
                // partner already an active single press → late chord upgrade
                sm->b[i].in_chord = true;
                sm->b[other].in_chord = true;
                emit(sm, (btn_id_e)other, BEV_UP, now);
                sm->b[other].reported = false;
                sm->b[other].held = false;
                sm->chord_active = true;
                emit(sm, BTN_UP, BEV_CHORD_UPDOWN, now);
            } else {
                sm->b[i].pending = true;               // open chord window
            }
        }
    }

    // chord window expiry → promote pendings to real presses
    for (int i = 0; i < BTN_COUNT; i += 2) {
        if (sm->b[i].pending && now - sm->b[i].press_us >= sm->chord_us) {
            sm->b[i].pending = false;
            sm->b[i].reported = true;
            emit(sm, (btn_id_e)i, BEV_DOWN, sm->b[i].press_us);
        }
    }

    // ---- MODE button: immediate, never chords -------------------------------
    if (changed[BTN_MODE]) {
        if (sm->b[BTN_MODE].stable) {
            sm->b[BTN_MODE].press_us = now;
            sm->b[BTN_MODE].reported = true;
            emit(sm, BTN_MODE, BEV_DOWN, now);
        }
    }

    // ---- releases / hold / repeat ------------------------------------------
    for (int i = 0; i < BTN_COUNT; i++) {
        if (changed[i] && !sm->b[i].stable) {          // release edge
            if (sm->b[i].in_chord) {
                sm->b[i].in_chord = false;
                int other = (i == BTN_UP) ? BTN_DOWN : (i == BTN_DOWN ? BTN_UP : -1);
                if (sm->chord_active &&
                    (other < 0 || !sm->b[other].in_chord)) {
                    sm->chord_active = false;
                    emit(sm, BTN_UP, BEV_CHORD_UPDOWN_END, now);
                }
                continue;
            }
            if (sm->b[i].pending) {
                // released inside the chord window: it was a quick tap
                sm->b[i].pending = false;
                emit(sm, (btn_id_e)i, BEV_DOWN, sm->b[i].press_us);
                emit(sm, (btn_id_e)i, BEV_UP, now);
                emit(sm, (btn_id_e)i, BEV_SHORT, now);
                continue;
            }
            if (sm->b[i].reported) {
                emit(sm, (btn_id_e)i, BEV_UP, now);
                emit(sm, (btn_id_e)i,
                     sm->b[i].held ? BEV_HOLD_END : BEV_SHORT, now);
                sm->b[i].reported = false;
                sm->b[i].held = false;
            }
        }

        if (sm->b[i].reported && sm->b[i].stable && !sm->b[i].in_chord) {
            if (!sm->b[i].held && now - sm->b[i].press_us >= sm->hold_us) {
                sm->b[i].held = true;
                sm->b[i].last_repeat_us = now;
                emit(sm, (btn_id_e)i, BEV_HOLD, now);
            }
            if (sm->b[i].held && now - sm->b[i].last_repeat_us >= sm->repeat_us) {
                sm->b[i].last_repeat_us = now;
                emit(sm, (btn_id_e)i, BEV_REPEAT, now);
            }
        }
    }
}
