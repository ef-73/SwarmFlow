#!/bin/bash
# SwarmFlow CI referee (Lead-owned; design §14.3, AGENTS.md §6).
# Run from the repo root. To verify a task branch, run main's copy:  bash <(git show main:scripts/ci.sh)
#   (not `| bash -s`: docker compose run would read the rest of the script from stdin)
#   CI_BASE        base ref to diff against (default: main, or origin/$GITHUB_BASE_REF on GitHub)
#   CI_BRANCH      branch name (default: current branch / GitHub head ref)
#   CI_SKIP_DOCKER=1  run only the host-side git checks
#   CI_IN_CONTAINER=1 build/test steps run directly (already inside the dev image, e.g. GitHub Actions)
set -uo pipefail
export MSYS_NO_PATHCONV=1
cd "$(git rev-parse --show-toplevel)"

CONTRACT_PATHS=(src/swarmflow_interfaces/ layouts/schema/ src/swarmflow_core/swarmflow_core/api.py tests/fixtures/)
PY_ROOTS=(sim2d tools/layoutgen tools/scenarios tools/bench tools/metrics tests/unit)

BRANCH="${CI_BRANCH:-${GITHUB_HEAD_REF:-${GITHUB_REF_NAME:-$(git rev-parse --abbrev-ref HEAD)}}}"
if [ -n "${CI_BASE:-}" ]; then BASE="$CI_BASE"
elif [ -n "${GITHUB_BASE_REF:-}" ]; then BASE="origin/$GITHUB_BASE_REF"
else BASE=main; fi
MERGE_BASE="$(git merge-base "$BASE" HEAD 2>/dev/null || git rev-parse HEAD)"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-swarmflow-$(basename "$(pwd)")}"
export SWARMFLOW_GUI=none

fail=0
pass() { echo "  PASS  $1"; }
bad()  { echo "  FAIL  $1"; fail=1; }

changed="$(git diff --name-only "$MERGE_BASE" HEAD)"
echo "ci.sh: branch=$BRANCH base=$BASE merge-base=${MERGE_BASE:0:10} changed-files=$(echo "$changed" | grep -c . || true)"
echo "== host checks"

# 1. Contract-diff ---------------------------------------------------------------------------------
contract_hits=""
for p in "${CONTRACT_PATHS[@]}"; do
  hits="$(echo "$changed" | grep -E "^${p//./\\.}" || true)"
  [ -n "$hits" ] && contract_hits+="$hits"$'\n'
done
contract_hits="$(echo "$contract_hits" | grep . || true)"
if [ -z "$contract_hits" ]; then pass "contract-diff (no contract paths touched)"
elif [[ "$BRANCH" == lead/contract-* ]]; then pass "contract-diff (contract branch $BRANCH)"
elif [ "$BRANCH" = main ] || [ "$BRANCH" = HEAD ]; then pass "contract-diff (on $BRANCH, nothing to compare)"
else bad "contract-diff: $BRANCH touches frozen contracts:"; echo "$contract_hits" | sed 's/^/          /'; fi

# 2+3. Owned-files and lead-tests for task branches (ws-x/T###-slug) ---------------------------------
card_section() { # card_section <card> <heading>  → bullet items (backticks stripped)
  awk -v h="## $2" '$0==h{f=1;next} /^## /{f=0} f && /^- /{sub(/^- /,""); print}' "$1" | tr -d '`' | sed 's/ *(.*)$//'
}
glob_match() { # glob_match <path> <glob>  (** matches across /)
  local re; re="$(printf '%s' "$2" | sed -e 's/[.+^$(){}|]/\\&/g' -e 's/\*\*/\x01/g' -e 's/\*/[^\/]*/g' -e 's/?/[^\/]/g' -e 's/\x01/.*/g')"
  [[ "$1" =~ ^${re}$ ]] || [[ "$1" == "$2"/* ]] || [[ "$2" == */ && "$1" == "$2"* ]]
}
if [[ "$BRANCH" =~ ^ws-[a-f]/(T[0-9]{3})- ]]; then
  task="${BASH_REMATCH[1]}"
  card="$(ls docs/tasks/"$task"-*.md 2>/dev/null | head -1)"
  if [ -z "$card" ]; then bad "owned-files: no task card docs/tasks/$task-*.md"
  else
    # Lead base: the lead commits card + lead tests on the task branch before dispatch (so main stays green).
    # If that commit is not on the base branch, owned-files and lead-tests diff from it instead of the merge-base.
    TASK_BASE="$MERGE_BASE"
    card_commit="$(git log --format=%H --diff-filter=A "$MERGE_BASE"..HEAD -- "$card" | tail -1)"
    [ -n "$card_commit" ] && TASK_BASE="$card_commit"
    changed_task="$(git diff --name-only "$TASK_BASE" HEAD)"
    mapfile -t owned < <(card_section "$card" "Owned files")
    mapfile -t ltests < <(card_section "$card" "Lead tests (do not edit)" | grep -v '^none' || true)
    outside=""
    while IFS= read -r f; do
      [ -z "$f" ] && continue
      ok=0
      [ "$f" = "$card" ] && ok=1
      [[ "$f" =~ ^docs/agent_log/.*-$task\.md$ ]] && ok=1
      for g in "${owned[@]}"; do [ -n "$g" ] && glob_match "$f" "$g" && ok=1; done
      for t in "${ltests[@]}"; do [ "$f" = "$t" ] && ok=0; done
      [ $ok = 1 ] || outside+="$f"$'\n'
    done <<< "$changed_task"
    if [ -z "$outside" ]; then pass "owned-files ($card, base ${TASK_BASE:0:10})"; else bad "owned-files: outside $card:"; echo -n "$outside" | sed 's/^/          /'; fi
    lt_bad=""
    for t in "${ltests[@]}"; do
      [ -z "$t" ] && continue
      present="$(git ls-tree -r --name-only "$TASK_BASE" -- "$t")"
      [ -z "$present" ] && lt_bad+="$t(missing at lead base) "
      for f in $present; do
        git diff --quiet "$TASK_BASE" HEAD -- "$f" || lt_bad+="$f "
      done
    done
    if [ -z "$lt_bad" ]; then pass "lead-tests unchanged"; else bad "lead-tests modified: $lt_bad"; fi
  fi
else
  pass "owned-files / lead-tests (not a task branch)"
fi

# 4. Hygiene -----------------------------------------------------------------------------------------
hyg=""
while IFS= read -r f; do
  [ -f "$f" ] || continue
  sz=$(wc -c < "$f"); [ "$sz" -gt 5242880 ] && hyg+="  >5MB: $f"$'\n'
done < <(git ls-files)
git ls-files | grep -E '^runs/|\.db3$|\.mcap$' | sed 's/^/  forbidden: /' > /tmp/sf_hyg_$$ ; hyg+="$(cat /tmp/sf_hyg_$$)"; rm -f /tmp/sf_hyg_$$
crlf="$(git ls-files --eol | awk '$1 ~ /crlf/ {print $NF}' | grep -E '\.(sh|py|xml|xacro|sdf|yaml|yml|json|msg|srv|action|cfg|txt|md|launch\.py|Dockerfile)$|Dockerfile$' || true)"
[ -n "$crlf" ] && hyg+="$(echo "$crlf" | sed 's/^/  CRLF in index: /')"$'\n'
secrets="$(git grep -nIE 'ghp_[A-Za-z0-9]{20,}|sk-[A-Za-z0-9_-]{20,}|-----BEGIN [A-Z ]*PRIVATE KEY-----' -- . ':!scripts/ci.sh' || true)"
[ -n "$secrets" ] && hyg+="$(echo "$secrets" | sed 's/^/  secret? /')"$'\n'
hyg="$(echo "$hyg" | grep . || true)"
if [ -z "$hyg" ]; then pass "hygiene"; else bad "hygiene:"; echo "$hyg"; fi

# Container helper -----------------------------------------------------------------------------------
in_dev() {
  if [ "${CI_IN_CONTAINER:-0}" = 1 ]; then bash -c "source /opt/ros/jazzy/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; $1"
  elif [ "${lock_held:-0}" != 1 ]; then echo "  (skipped: no build lock)"; return 1
  else docker compose run --rm --no-deps -T dev bash -c "$1" </dev/null; fi
}

if [ "${CI_SKIP_DOCKER:-0}" = 1 ]; then
  echo "== container checks skipped (CI_SKIP_DOCKER=1)"
else
  echo "== container checks (dev image, project $COMPOSE_PROJECT_NAME)"
  lock_held=0
  if [ "${CI_IN_CONTAINER:-0}" != 1 ]; then
    for i in $(seq 1 120); do scripts/lock.sh acquire build ci 30 >/dev/null 2>&1 && { lock_held=1; break; }; sleep 10; done
    [ $lock_held = 1 ] || bad "could not acquire build lock in 20 min"
    trap '[ $lock_held = 1 ] && scripts/lock.sh release build >/dev/null' EXIT
  fi

  # 5. Generated layouts fresh
  if [ -f tools/layoutgen/generate.py ] && ls layouts/*/layout.yaml >/dev/null 2>&1; then
    if in_dev 'set -e; for y in layouts/*/layout.yaml; do d=$(dirname $y); python3 tools/layoutgen/generate.py "$y" --out "$d/generated"; done' >/tmp/sf_gen_$$ 2>&1 \
       && git diff --exit-code --stat -- 'layouts/*/generated' && [ -z "$(git ls-files --others --exclude-standard -- 'layouts/*/generated')" ]; then
      pass "generated layouts fresh"
    else bad "generated layouts stale or generator failed"; tail -20 /tmp/sf_gen_$$; fi
    rm -f /tmp/sf_gen_$$
  else pass "generated layouts (no generator yet)"; fi

  # 6. colcon build + test
  if [ -n "$(find src -name package.xml -print -quit)" ]; then
    if in_dev 'export MAKEFLAGS=-j2; colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+ 2>&1 | tail -40; exit ${PIPESTATUS[0]}'; then
      pass "colcon build"
      # ROS node tests share one DDS domain inside the container: run packages one at a time (parallel runs cross-talk on /fleet/*).
      # Fresh results only: a crashed or timed-out run must not be judged by old result files (M9 review C1).
      if in_dev 'source install/setup.bash; find build -path "*/test_results/*" -delete 2>/dev/null; find build -name "pytest.xml" -delete 2>/dev/null; timeout 1200 colcon test --parallel-workers 1 --event-handlers console_direct- >/dev/null 2>&1; rc=$?; colcon test-result --verbose | tail -40; n=$(colcon test-result 2>/dev/null | sed -n "s/^Summary: \([0-9]*\) tests.*//p"); [ "$rc" = 0 ] && [ "${n:-0}" -gt 0 ] && colcon test-result >/dev/null'; then
        pass "colcon test"
      else bad "colcon test"; fi
    else bad "colcon build"; fi
  else pass "colcon (no packages yet)"; fi

  # 6b. Static checks of the runtime config (M9 review C2): compose file, xacro, launch files
  if docker compose config -q 2>/dev/null || [ "${CI_IN_CONTAINER:-0}" = 1 ]; then pass "compose config"; else bad "compose config"; fi
  if [ -f src/swarmflow_description/urdf/swarmflow_bot.urdf.xacro ]; then
    if in_dev 'xacro src/swarmflow_description/urdf/swarmflow_bot.urdf.xacro robot_id:=robot_1 namespace:=robot_1 > /dev/null && for f in src/*/launch/*.py; do python3 -m py_compile "$f" || exit 1; done'; then
      pass "xacro renders, launch files compile"
    else bad "xacro / launch files"; fi
  fi

  # 7. pytest over pure-Python roots
  roots=""; for r in "${PY_ROOTS[@]}"; do [ -d "$r" ] && roots+="$r "; done
  if [ -n "$roots" ]; then
    if in_dev "python3 -m pytest -q -p no:cacheprovider --import-mode=importlib $roots 2>&1 | tail -25; exit \${PIPESTATUS[0]}"; then pass "pytest ($roots)"
    else bad "pytest ($roots)"; fi
  else pass "pytest (no pure-Python roots yet)"; fi
  # 8. test_no_ros_imports runs inside colcon test of swarmflow_core.
fi

echo
if [ $fail = 0 ]; then echo "ci.sh: GREEN"; else echo "ci.sh: RED"; fi
exit $fail
