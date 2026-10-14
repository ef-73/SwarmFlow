## 2026-10-14T07:50Z · claude · Lead · M14 N-09 keep-right · DONE (in sim)
- Branch: lead/T038-nav-blocked
- Model / attempts: lead; 2 sessions (s13 died at start: YAML aliases in the generated params file, fixed)
- Did: KeepRightLayer (left half of each of the 11 layout aisles, by heading, cost 70); lane share in nav_scenarios.
- Verified (Gazebo s14): lane_n / lane_s 4/4 with 95–100 % of central-aisle samples on the right half (≥ 80 %
  required); head_on 24–26 s (was ~31 s), robots on their right halves (0.87 / 1.0), min 0.29 m; pass_parked
  0.26 m; dead_end / dead_end_rev resolve in 21–24 s (padded overlap ≤ 2.3 cm, unchanged).
- Sim used: yes, 07:40–07:47 UTC, lock held and released.
- Unverified / assumptions: KeepRightLayer assumes the costmap frame equals the map frame (true with ground-truth
  localization); with AMCL (I-01) the aisles must be transformed like OtherRobotsLayer does.
- Needs user: nothing.
