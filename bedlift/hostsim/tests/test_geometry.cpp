// Geometry / polarity regression test — the guard rail for the single source
// of truth (bed_geometry.h). Links only control_law.c + the header, no sim /
// cybergear / SDL, so it always builds and runs fast (ctest: `geometry`).
//
// It pins four things that kept drifting out of sync by hand:
//   1. the motion vectors (lift/pitch/roll/twist) match the corner table
//   2. level_control drives every corner the RIGHT way for a tilt error
//   3. front/rear roll is independent (a front-only tilt never moves the rear)
//   4. travel_trim agrees in sign with level_control
//   5. closed loop: feeding level_control into a plant built from the SAME
//      table drives the bed to level — any sign inversion diverges and fails.

#include <cstdio>
#include <cmath>
#include "bed_geometry.h"
#include "control_law.h"

static int g_fail = 0;
#define CHECK(cond, msg, ...)                                           \
    do { if (!(cond)) { g_fail++;                                       \
        printf("  FAIL: " msg "\n", ##__VA_ARGS__); }                   \
        else printf("  ok:   " msg "\n", ##__VA_ARGS__); } while (0)

static tilt_snap_t snap(float pitch, float roll)
{
    tilt_snap_t t;
    t.pitch_deg = pitch; t.roll_deg = roll; t.valid = true; t.t_us = 0;
    return t;
}

// ---- 1. motion vectors match the table ------------------------------------
static void test_vectors(void)
{
    printf("[vectors]\n");
    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        CHECK(bed_axis_vec(BED_AXIS_LIFT, i) == 1.0f, "LIFT[%d] = +1", i);
        // pitch raises fronts, lowers backs
        CHECK(bed_axis_vec(BED_AXIS_PITCH, i) == (bed_is_front(i) ? 1.0f : -1.0f),
              "PITCH[%d] %s", i, bed_is_front(i) ? "front up" : "back down");
        // roll raises rights, lowers lefts
        CHECK(bed_axis_vec(BED_AXIS_ROLL, i) == (bed_lr(i) > 0 ? 1.0f : -1.0f),
              "ROLL[%d] %s", i, bed_lr(i) > 0 ? "right up" : "left down");
        // twist = front axle only, right up / left down; rear holds
        float want = bed_is_front(i) ? (bed_lr(i) > 0 ? 1.0f : -1.0f) : 0.0f;
        CHECK(bed_axis_vec(BED_AXIS_TWIST, i) == want, "TWIST[%d]", i);
    }
}

// ---- 2/3. level_control drives the right way, front/rear independent -------
static void test_level_signs(void)
{
    level_law_t l; level_law_init(&l);
    float v[SYS_NUM_MOTORS];

    printf("[level: +pitch (front high) -> lower front, raise back]\n");
    tilt_snap_t p = snap(5, 0);
    level_control(&l, &p, &p, v);
    for (int i = 0; i < SYS_NUM_MOTORS; i++)
        CHECK(bed_is_front(i) ? v[i] < 0 : v[i] > 0,
              "M%d (%s) v=%+.3f", i, bed_is_front(i) ? "front" : "back", v[i]);

    printf("[level: +roll (right high) -> lower right, raise left]\n");
    tilt_snap_t r = snap(0, 5);
    level_control(&l, &r, &r, v);
    for (int i = 0; i < SYS_NUM_MOTORS; i++)
        CHECK(bed_lr(i) > 0 ? v[i] < 0 : v[i] > 0,
              "M%d (%s) v=%+.3f", i, bed_lr(i) > 0 ? "right" : "left", v[i]);

    printf("[level: front-only roll never moves the rear pair]\n");
    tilt_snap_t f_roll = snap(0, 5), level = snap(0, 0);
    level_control(&l, &f_roll, &level, v);
    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        if (bed_is_front(i)) CHECK(v[i] != 0, "front M%d moves (%+.3f)", i, v[i]);
        else                 CHECK(v[i] == 0, "rear  M%d holds (%+.3f)", i, v[i]);
    }
}

// ---- 4. travel_trim agrees in sign with level_control ----------------------
static void test_trim_agrees(void)
{
    printf("[trim: same direction as level_control]\n");
    level_law_t l; level_law_init(&l);
    tilt_snap_t f = snap(4, 3), rr = snap(4, -2);
    float lv[SYS_NUM_MOTORS], tv[SYS_NUM_MOTORS];
    level_control(&l, &f, &rr, lv);
    travel_trim(&l, &f, &rr, 1.0f, tv);
    for (int i = 0; i < SYS_NUM_MOTORS; i++)
        CHECK((lv[i] > 0) == (tv[i] > 0) && (lv[i] < 0) == (tv[i] < 0),
              "M%d level=%+.3f trim=%+.3f", i, lv[i], tv[i]);
}

// ---- 5. closed-loop convergence against a plant built from the same table --
// Plant: corner height integrates commanded velocity; tilt is read back with
// bed_geometry's fb/lr. If any control sign is wrong the bed drives itself
// MORE unlevel and the final error assert fails.
#define TRACK 1.2f
#define LENGTH 2.0f
static void plant_tilt(const float h[SYS_NUM_MOTORS], float *pitch,
                       float *roll_f, float *roll_r)
{
    float fr = 0, ba = 0, frs = 0, rrs = 0;  // front/back sums; front/rear roll
    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        if (bed_fb(i) > 0) fr += h[i]; else ba += h[i];
        float s = (bed_lr(i) > 0 ? +h[i] : -h[i]);
        if (bed_is_front(i)) frs += s; else rrs += s;
    }
    *pitch  = atan2f((fr - ba) / 2.0f, LENGTH) * 57.2958f;
    *roll_f = atan2f(frs, TRACK) * 57.2958f;
    *roll_r = atan2f(rrs, TRACK) * 57.2958f;
}

static void test_convergence(void)
{
    printf("[closed loop: disturbed bed levels itself]\n");
    level_law_t l; level_law_init(&l);
    // seed corner heights that produce pitch=+5, front roll=+4, rear roll=-3
    // (a real twist) — solve directly from the table.
    float h[SYS_NUM_MOTORS] = {0};
    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        h[i]  = bed_fb(i) * (LENGTH * tanf(5.0f / 57.2958f) / 2.0f);
        float rr = bed_is_front(i) ? 4.0f : -3.0f;
        h[i] += bed_lr(i) * (TRACK * tanf(rr / 57.2958f) / 2.0f);
    }
    float p0, rf0, rr0; plant_tilt(h, &p0, &rf0, &rr0);
    printf("  start: pitch=%+.2f rollF=%+.2f rollR=%+.2f\n", p0, rf0, rr0);

    const float dt = 0.001f;
    float pitch = p0, rf = rf0, rr = rr0;
    for (int step = 0; step < 40000; step++) {
        plant_tilt(h, &pitch, &rf, &rr);
        tilt_snap_t sf = snap(pitch, rf), sr = snap(pitch, rr);
        float v[SYS_NUM_MOTORS];
        bool done = level_control(&l, &sf, &sr, v);
        for (int i = 0; i < SYS_NUM_MOTORS; i++) h[i] += v[i] * dt;
        if (done) break;
    }
    plant_tilt(h, &pitch, &rf, &rr);
    printf("  end:   pitch=%+.2f rollF=%+.2f rollR=%+.2f\n", pitch, rf, rr);
    CHECK(fabsf(pitch) < 0.6f, "pitch converged (%.2f)", pitch);
    CHECK(fabsf(rf) < 0.6f, "front roll converged (%.2f)", rf);
    CHECK(fabsf(rr) < 0.6f, "rear roll converged (%.2f)", rr);
    // and it must have IMPROVED, not just ended small by luck
    CHECK(fabsf(pitch) < fabsf(p0) && fabsf(rf) < fabsf(rf0),
          "error decreased from start");
}

int main(void)
{
    printf("=== bed geometry / polarity tests ===\n");
    printf("layout: idx0=%s idx1=%s idx2=%s idx3=%s\n",
           "FR", "BR", "FL", "BL");
    test_vectors();
    test_level_signs();
    test_trim_agrees();
    test_convergence();
    printf("=== %s (%d failures) ===\n", g_fail ? "FAILED" : "PASSED", g_fail);
    return g_fail ? 1 : 0;
}
