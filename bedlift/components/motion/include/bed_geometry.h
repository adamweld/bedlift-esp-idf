#ifndef BEDLIFT_BED_GEOMETRY_H
#define BEDLIFT_BED_GEOMETRY_H

// ===========================================================================
// SINGLE SOURCE OF TRUTH for motor<->corner geometry and motion polarity.
//
// Every motion vector (lift / pitch / roll / twist), the leveling law, the
// travel trim, AND the simulator's plant geometry are DERIVED from the one
// table below (BED_CORNER). To re-map a motor to a different corner, or flip
// a whole side, edit ONLY this table — nothing else hardcodes the layout.
//
// Frame axes are bed-fixed and match the IMU calibration in sensors.c:
//     fb = +1 FRONT / -1 BACK      (+pitch = front / nose high)
//     lr = +1 RIGHT / -1 LEFT      (+roll  = right side high)
//
// Anchored on the bench 2026-09-23 (both are MEASUREMENTS, not conventions):
//   * fb from the VERIFIED pitch vector {+1,-1,+1,-1}: driving idx0,idx2 up
//     raises the nose, so idx0,idx2 are the FRONT pair.
//   * lr from the physical fact that idx2,idx3 sit on the LEFT rail, so
//     idx0,idx1 are the RIGHT rail.
//
// Resulting layout (idx = CyberGear M-number - 1: M1=idx0 ... M4=idx3):
//     idx0 = M1 = front-right     idx1 = M2 = back-right
//     idx2 = M3 = front-left      idx3 = M4 = back-left
//
// The ONE remaining hardware check is roll polarity: raising the RIGHT rail
// (idx0,idx1) must read +roll on the IMU. If a roll/self-level test shows the
// frame driving itself MORE unlevel, flip both lr signs below and pitch, roll,
// level, trim, and the sim all follow coherently from this one edit.
// ===========================================================================

#include "sys_state.h"   // SYS_NUM_MOTORS

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int fb; int lr; } bed_corner_t;

static const bed_corner_t BED_CORNER[SYS_NUM_MOTORS] = {
    { +1, +1 },   // idx0 = M1 = front-right
    { -1, +1 },   // idx1 = M2 = back-right
    { +1, -1 },   // idx2 = M3 = front-left
    { -1, -1 },   // idx3 = M4 = back-left
};

typedef enum {
    BED_AXIS_LIFT = 0,   // every corner up
    BED_AXIS_PITCH,      // front up / back down   (drives +pitch)
    BED_AXIS_ROLL,       // right up / left down    (drives +roll)
    BED_AXIS_TWIST,      // front axle only: right up / left down, rear holds
} bed_axis_e;

// Per-motor direction (+1 = this corner drives UP) for one motion axis.
// This is the ONLY place motion vectors come from — manual PITCH/ROLL/TWIST
// buttons and the LIFT group all read this.
static inline float bed_axis_vec(bed_axis_e ax, int i)
{
    const bed_corner_t c = BED_CORNER[i];
    switch (ax) {
        case BED_AXIS_LIFT:  return 1.0f;
        case BED_AXIS_PITCH: return (float)c.fb;
        case BED_AXIS_ROLL:  return (float)c.lr;
        case BED_AXIS_TWIST: return c.fb > 0 ? (float)c.lr : 0.0f;
    }
    return 0.0f;
}

static inline int  bed_fb(int i)       { return BED_CORNER[i].fb; }
static inline int  bed_lr(int i)       { return BED_CORNER[i].lr; }
static inline bool bed_is_front(int i) { return BED_CORNER[i].fb > 0; }

#ifdef __cplusplus
}
#endif

#endif
