# Nav graph subset (layouts/<name>/generated/nav_graph.yaml)

Frozen contract (M2). SwarmFlow's layout generator writes the nav graph in **Open-RMF's nav-graph YAML format**
(design §5.4) so Baseline C (v2) can load it unchanged. Only the subset below is emitted; SwarmFlow-only data lives
in `zones.yaml` (`zones.schema.json`).

Format checked on 2026-10-09 against `rmf_building_map_tools` (`jazzy` branch): `building_map/building.py`
`generate_nav_graphs()` (top-level keys) and `building_map/level.py` `generate_nav_graph()` (vertex/lane lists,
`always_unidirectional=True` splits two-way lanes). Loading by RMF itself is still **[U]** until Baseline C (v2).

```yaml
building_name: <layout name>
doors: {}
lifts: {}
levels:
  L1:                                   # single level, always "L1"
    vertices:                           # index = position in this list
      - [x, y, {name: "<vertex name>", <params>}]
    lanes:                              # one entry per direction ([from_index, to_index, params])
      - [i, j, {speed_limit: <m/s>}]    # speed_limit only if > 0 (as upstream)
```

Vertex params emitted (RMF traffic-editor vertex params, passed through upstream as-is):

| Param | When | Value |
|---|---|---|
| `name` | always | SwarmFlow vertex name (`L1`, `D2`, `P1`, `X_1_3`, `H_1_1`; v2: `PK_W3_N_04`, `DK2`, `Q_DK2_1`, `HM3`, `PL_1_2`) |
| `is_parking_spot` | park stations `P*`; v2 home slots `HM*` | `true` |
| `is_charger` | park stations `P*`; v2 home slots `HM*` | `true` (no batteries; RMF needs a charger per robot) |
| `is_holding_point` | hold vertices `H_*` | `true` |
| `pickup_dispenser` / `dropoff_ingestor` | never in v1 | — (RMF workcells unused, design §5.5) |

Rules:
- Vertex order: stations (L, D, P in name order; v2 layouts then DK, HM, PK, PL, Q_ groups, each in name order), then
  intersections, then holds, each sorted by name. Indices are
  derived from this order; SwarmFlow code always refers to vertices **by name**, never by index.
- Lanes: authored bidirectional lanes become two entries (`[a,b]`, `[b,a]`); one-way lanes one entry. Sorted by
  (from name, to name).
- Coordinates are in the map frame in metres (RMF `building.yaml` uses pixels + scale; the generated nav graph
  is already in metres).
