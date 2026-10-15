// Cost of another robot's predicted footprint `t` seconds ahead (OtherRobotsLayer, N-05).
// 0 .. near_s: from `predicted` down to half of it (the 2 s prediction as recorded in the v2 benchmarks);
// near_s .. horizon_s: fades further, from half down to `far`, so a long look-ahead nudges paths instead of
// closing a narrow aisle (decided with the user 2026-10-15: test a 5 s look-ahead with fading cost).
#pragma once
#include <algorithm>
#include <cmath>

namespace swarmflow_nav
{
inline unsigned char predictedCost(double t, double near_s, double horizon_s, unsigned char predicted, unsigned char far)
{
  const double half = predicted / 2.0;
  double c;
  if (t <= near_s || horizon_s <= near_s) {
    c = predicted - half * std::clamp(t / std::max(near_s, 1e-6), 0.0, 1.0);
  } else {
    c = half + (std::min<double>(far, half) - half) * std::clamp((t - near_s) / (horizon_s - near_s), 0.0, 1.0);
  }
  return static_cast<unsigned char>(std::lround(std::clamp(c, 1.0, 252.0)));
}
}  // namespace swarmflow_nav
