# 001 — Ground-truth localization by default (design §15.3 cut 1)

- **Date:** 2026-10-09 (M4) · **Decided by:** lead (autonomous run; the user can overrule)
- **Decision:** robots localize with Gazebo ground truth by default (`SWARMFLOW_LOCALIZATION=ground_truth`):
  `odom → base_footprint` from Gazebo's `OdometryPublisher` (absolute world pose), `map → odom` identity.
  AMCL stays available (`SWARMFLOW_LOCALIZATION=amcl`) but is not part of the v1 acceptance runs.
- **Why:** (1) the first single-robot session showed DiffDrive wheel odometry drifting by metres after wheel slip, so
  "static map→odom + wheel odometry" was not usable; (2) three AMCL instances cost CPU on a thermally limited laptop
  (design §8.5 item 4); (3) localization research is a non-goal (design §2.3).
- **Consequence:** v1 does not demonstrate localization; navigation, the fleet layer and the traffic scene are
  unaffected. Listed in the README under "known limitations".
