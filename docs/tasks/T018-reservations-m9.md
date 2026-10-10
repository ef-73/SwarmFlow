---
id: T018
title: Reservation authority hardening — owner re-grant, unauthorized entry, FIFO queue
workstream: B
status: review
claimed_by: "sf-implementer"
branch: ws-b/T018b-reservations-m9
model: sonnet
risk: safety-critical
---

## Goal
Fix three findings of the M9 independent review in `FcfsReservationAuthority`, without changing `api.py`.

## Owned files
- src/swarmflow_core/swarmflow_core/reservations.py
- src/swarmflow_core/test/test_reservations_extra.py

## Lead tests (do not edit)
- src/swarmflow_core/test/test_reservations.py
- src/swarmflow_core/test/test_reservations_m9.py

## Forbidden
- AGENTS.md §4; do not edit api.py or any other file.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t018
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_core/test tools sim2d
```

## Specification
1. **Owner re-grant (S2, liveness):** a robot stuck inside a zone gets its own blocking lease (FAULT release with an
   inside pose); its park task then requests that zone and was denied forever. New rule in `request`: if the zone has
   no GRANTED lease of another robot and **every** blocking lease in the zone belongs to the requester, then each of
   those blocking leases ends as `RELEASED` with reason `SUPERSEDED` (event) and the requester gets a new GRANTED lease
   (normal TTL/hard bound). Otherwise unchanged.
2. **Unauthorized entry (S3, safety):** in `observe_robot` (newest-stamp samples only), if the pose is inside a zone
   polygon and the robot holds no live lease (GRANTED or blocking) in that zone, create a blocking lease
   `unauthorized_<robot>_<zone>_<n>` (state OCCUPIED_UNKNOWN, reason `UNAUTHORIZED_ENTRY`, granted_t = expiry_t =
   hard_expiry_t = blocking_since = t), record the event and return it. It is cleared like any blocking lease
   (owner observed outside, `clear_zone`). Never create a duplicate for the same robot and zone.
3. **FIFO queue (S6, fairness — "FCFS" must be first come, first served):** per zone, remember robots that were denied
   (any deny except UNKNOWN_ZONE / BAD_ENTRY / RECOVERING) in order of their first denial, with the time of their
   latest request. A robot that is not at the head of the queue is denied with reason `QUEUED_BEHIND:<head robot>`
   while the head's latest request is younger than `QUEUE_STALE_S = 2.5` s (agents retry about every 1 s); a stale
   head is dropped. A robot leaves the queue when it is granted. Idempotent re-grants are unaffected.
4. Module docstring: document the three rules and why (M9 review S2, S3, S6).
- The lead updated `test_reservations.py`'s property test: the invariant is now "never a NEW grant while another
  robot blocks the zone" (an intruder may legitimately block a zone someone else holds), and hypothesis runs are
  `derandomize=True` (reproducible in CI). Update your own extra tests if the new rules change them.

- Unauthorized-entry leases **must** appear in `pop_events` (they are broadcast on `/fleet/reservations`); the
  lead's property test now allows a first event OCCUPIED_UNKNOWN for reasons RECOVERED / UNAUTHORIZED_ENTRY.

## Definition of done
- Both lead test files pass unchanged; all of `src/swarmflow_core/test`, `tools`, `sim2d` pass (the 2D corridor tests
  use the authority). Commit "T018: reservation hardening", status `review`. Report test output.

## Inputs / contracts used
- api.py `ReservationAuthority` docstring; design §6.5; M9 review findings S2, S3, S6 (quoted above).
