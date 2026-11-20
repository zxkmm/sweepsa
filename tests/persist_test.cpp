// Host tests for the persistence display's histogram: where hits land, that
// an intermittent signal shows as a proportionally dimmer layer, that strokes
// are joined at narrow spans, and that decay is in seconds, not sweeps.

#include <cmath>
#include <cstdio>
#include <vector>

#include "persist.h"

using namespace sweepui;

namespace {

int g_failures = 0;

void check_true(const char *what, bool ok) {
    std::printf("  %-62s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

void check_near(const char *what, double got, double want, double tol) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("  %-62s %s (got %.4f, want %.4f +/-%.4f)\n", what, ok ? "ok" : "FAIL",
                got, want, tol);
    if (!ok) ++g_failures;
}

// 100 columns over 0..100 Hz, 100 rows over -100..0 dB: one row per dB.
void lay(Persistence *p) { p->layout(100, 100, 0, 100, -100, 0); }

void test_placement_and_density() {
    std::printf("placement\n");
    Persistence p;
    lay(&p);
    p.set_tau(0);
    std::vector<float> floor(100, -80.0f), burst(100, -80.0f);
    burst[50] = -20.0f;
    // The burst is there in 1 sweep of 10.
    for (int k = 0; k < 10; ++k) p.add(k == 3 ? burst.data() : floor.data(), 100, 0, 1, 0.1);
    check_near("floor row hit every sweep", p.hits(10, 20), 10.0, 1e-9);
    check_near("burst level hit once", p.hits(50, 80), 1.0, 1e-9);
    check_true("cells far from any stroke stay dark", p.hits(10, 70) == 0.0);
    std::vector<float> in;
    p.intensity(&in);
    // Row 0 of the intensity is the top (0 dB).
    const float i_floor = in[size_t(99 - 20) * 100 + 10], i_burst = in[size_t(99 - 80) * 100 + 50];
    check_true("the rare burst is visible but dimmer than the floor",
               i_burst > 0.1f && i_burst < i_floor);
}

void test_joined_at_narrow_span() {
    std::printf("joined strokes\n");
    Persistence p;
    lay(&p);
    // 5 points over 100 columns, at columns 10..90: without joining, 76 of
    // the 81 columns between them would be empty.
    std::vector<float> v = {-90, -90, -10, -90, -90};
    p.add(v.data(), v.size(), 0, 20, 0);
    size_t lit = 0;
    for (size_t c = 10; c <= 90; ++c) {
        bool any = false;
        for (size_t r = 0; r < 100; ++r) any = any || p.hits(c, r) > 0;
        lit += any;
    }
    check_true("every column between the first and last point is lit", lit == 81);
    bool mid = false;
    for (size_t c = 30; c <= 50; ++c) mid = mid || p.hits(c, 50) > 0;
    check_true("the rise to the peak passes through the levels between", mid);
}

void test_decay_is_in_seconds() {
    std::printf("decay\n");
    std::vector<float> v(100, -50.0f), quiet(100, std::nanf(""));
    Persistence a, b;
    lay(&a);
    lay(&b);
    a.set_tau(1.0);
    b.set_tau(1.0);
    a.add(v.data(), 100, 0, 1, 0);
    b.add(v.data(), 100, 0, 1, 0);
    // One second as one sweep, or as ten: the same fade.
    a.add(quiet.data(), 100, 0, 1, 1.0);
    for (int k = 0; k < 10; ++k) b.add(quiet.data(), 100, 0, 1, 0.1);
    check_near("one 1 s step fades to 1/e", a.hits(5, 50), std::exp(-1.0), 1e-9);
    check_near("ten 0.1 s steps fade the same", b.hits(5, 50), a.hits(5, 50), 1e-9);
    check_true("NaN points light nothing", a.hits(5, 10) == 0.0);
}

}  // namespace

int main() {
    std::printf("persistence tests\n\n");
    test_placement_and_density();
    std::printf("\n");
    test_joined_at_narrow_span();
    std::printf("\n");
    test_decay_is_in_seconds();
    std::printf("\n%s\n", g_failures ? "FAILURES" : "all passed");
    return g_failures ? 1 : 0;
}
