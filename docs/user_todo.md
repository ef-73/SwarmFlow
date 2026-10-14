# Things for the user to do (non-blocking)

Small actions only the user can or should do. None of them blocks the lead's work. The lead adds items here
instead of asking in chat; tick them off or delete the line when done.

| Added | Item | Why the lead can't | How |
|---|---|---|---|
| 2026-10-14 | **Re-import the Foxglove layout** `viz/foxglove/swarmflow_v2.json`: your account's copy still has the v1 order template in the Publish panel (v2 orders need `order_type`, `pick_vertices`, `dock`). | Your Foxglove account. | Foxglove → layout menu → *Import from file* → `viz/foxglove/swarmflow_v2.json`; delete the older copy. |
| 2026-10-14 | **Review decision 003** (composed Nav2 never applied costmap parameters; v1 ran on Nav2 defaults since M4). | Affects how the v1 results read. | `docs/decisions/003-composed-nav2-costmap-params.md` |
| 2026-10-11 | **Delete duplicate Foxglove layouts.** Your Foxglove account has several layouts named `swarmflow_v2` from the T021/T022 imports. Keep the newest one (it opens with frame labels hidden), delete the others. | Deleting data in your account is yours to do. | Foxglove → layout menu (top right) → *Manage layouts* → delete the older `swarmflow_v2` entries. |
| 2026-10-10 | **Answer or dismiss the "Docker Desktop Backend" firewall prompt** (if it is still open). | Security setting. | Either answer works; the ports are bound to `127.0.0.1`. |
| 2026-10-10 | **Optional: record a short demo video** and link it in the README (v1 release step). | Needs your screen and account. | Run `docker compose --profile gui up`, record Foxglove and/or the Gazebo window. |
