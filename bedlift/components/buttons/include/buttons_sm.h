#ifndef BEDLIFT_BUTTONS_SM_H
#define BEDLIFT_BUTTONS_SM_H

// Pure button state machine: debounce, up+down chord detection, press vs
// hold, repeat. Platform-free — feed it raw (active-true) levels and a
// monotonic timestamp; pop events. The ESP ISR/task wrapper and the hostsim
// keyboard both drive this same code.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { BTN_UP = 0, BTN_MODE = 1, BTN_DOWN = 2, BTN_COUNT = 3 } btn_id_e;

typedef enum {
    BEV_DOWN,       // press confirmed (post-debounce, post-chord-window)
    BEV_UP,         // release
    BEV_SHORT,      // released before hold threshold
    BEV_HOLD,       // held past threshold (fires once)
    BEV_HOLD_END,   // released after a HOLD fired
    BEV_REPEAT,     // periodic while held (after HOLD)
    BEV_CHORD_UPDOWN,      // up+down pressed together
    BEV_CHORD_UPDOWN_END,  // chord released (both up)
} btn_event_e;

typedef struct {
    btn_id_e id;        // undefined for chord events
    btn_event_e type;
    int64_t t_us;
} btn_event_t;

typedef struct {
    // timing (us) — defaults set by btn_sm_init, override after if needed
    int64_t debounce_us;
    int64_t chord_us;       // window for the two presses to count as one chord
    int64_t chord_hold_us;  // both must stay held this long before CHORD fires
    int64_t hold_us;
    int64_t repeat_us;

    // internal
    struct {
        bool raw, stable;
        int64_t last_edge_us;
        int64_t press_us;
        bool pending;       // in chord window (UP/DOWN only)
        bool reported;      // BEV_DOWN emitted
        bool held;          // BEV_HOLD emitted
        int64_t last_repeat_us;
        bool in_chord;
    } b[BTN_COUNT];
    bool chord_active;
    int64_t chord_start_us; // both-held-since; 0 = not pending
    bool wake_latch;        // swallow the deep-sleep wake press

    btn_event_t q[16];
    int q_head, q_tail;
} btn_sm_t;

void btn_sm_init(btn_sm_t *sm);
// Start latched: no events until all buttons are released once (wake press).
void btn_sm_init_latched(btn_sm_t *sm);

// Feed current levels (true = pressed) at any rate >= ~200 Hz.
void btn_sm_step(btn_sm_t *sm, bool up, bool mode, bool down, int64_t now_us);

// Pop the next event; false when empty.
bool btn_sm_poll(btn_sm_t *sm, btn_event_t *out);

#ifdef __cplusplus
}
#endif

#endif
