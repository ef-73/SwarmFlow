# Foxglove layouts

- **`swarmflow_v2.json`** (T021, current): import it from the Layouts menu (*Import from file*) after connecting to
  `ws://localhost:8765`. Checked by import in app.foxglove.dev. Panels and data: see the README section *Foxglove*.
- `swarmflow_v1.json` (v1, superseded): written from memory in M3 and never verified by import; kept for reference.

Topics used by v2: `/viz/scene` (latched world), `/viz/zones`, `/viz/packages`, `/viz/robots`, `/viz/robot_N/scan`,
`/viz/fleet_dashboard` (`diagnostic_msgs/DiagnosticArray`), `/fleet/order_status`, `/fleet/decisions`,
`/fleet/orders` (publish), `/tf`, `/tf_static`.

Layout keys verified in the app (2026-10-11): `scene.transforms.visible: false` hides the TF frame axes and labels;
LaserScan topics take `colorMode: "flat"` and `flatColor`.
