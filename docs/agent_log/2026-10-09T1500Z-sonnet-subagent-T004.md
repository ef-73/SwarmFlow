## 2026-10-09T15:00Z · sonnet-subagent · B · T004 · DONE
- Branch: ws-b/T004-fleet-core
- Model / attempts: sonnet, 1 attempt
- Did: FcfsPolicy, decisions.render/make_decision/reservation_decision, FleetCore (order book, lifecycle, stuck/park rules), FakeBackend + run_fake using the real reservation authority; extra tests.
- Verified: docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_core/test -> 88 passed
- Sim used: no
- Unverified / assumptions: scripts/ci.sh not run
- Needs user: nothing
