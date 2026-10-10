#!/bin/bash
# Atomic mkdir lock shared by every worktree and shell (AGENTS.md §5, design §14.5).
#   scripts/lock.sh acquire <name> [task-id] [expected-minutes]
#   scripts/lock.sh release <name> [--force]   (only the worktree that acquired it, unless --force)
#   scripts/lock.sh status  <name>
# Lock dir: $SWARMFLOW_LOCK_DIR/<name>.lock with owner.txt. Default lock root = the Windows user's
# ~/.swarmflow (C:\Users\<user>\.swarmflow; /mnt/c/Users/<user>/.swarmflow from WSL).
set -euo pipefail

lock_root() {
  if [ -n "${SWARMFLOW_LOCK_DIR:-}" ]; then echo "$SWARMFLOW_LOCK_DIR"; return; fi
  case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) echo "$HOME/.swarmflow" ;;
    *)
      if command -v cmd.exe >/dev/null 2>&1 && command -v wslpath >/dev/null 2>&1; then
        echo "$(wslpath "$(cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null | tr -d '\r')")/.swarmflow"
      else
        echo "$HOME/.swarmflow"
      fi ;;
  esac
}

cmd="${1:-}"; name="${2:-}"
[ -n "$cmd" ] && [ -n "$name" ] || { echo "usage: $0 acquire|release|status <name> [task] [minutes]" >&2; exit 2; }
root="$(lock_root)"; dir="$root/$name.lock"
tokdir="$(git rev-parse --show-toplevel 2>/dev/null || pwd)/.run"   # per-worktree ownership token (.run/ is gitignored)
mkdir -p "$root"

age_min() { # minutes since lock creation
  local now created; now=$(date +%s); created=$(stat -c %Y "$dir" 2>/dev/null || echo "$now")
  echo $(( (now - created) / 60 ))
}

case "$cmd" in
  acquire)
    task="${3:-unknown}"; minutes="${4:-20}"
    stale="${SWARMFLOW_LOCK_STALE_MIN:-60}"   # sim locks are checked by sim_lock.sh (30 min + container check)
    if [ "$name" != sim ] && [ -d "$dir" ] && [ "$(age_min)" -ge "$stale" ]; then
      echo "lock $name is STALE ($(age_min) min >= $stale), removing:" >&2; cat "$dir/owner.txt" >&2 2>/dev/null || true
      rm -rf "$dir"
    fi
    if mkdir "$dir" 2>/dev/null; then
      {
        echo "agent: ${SWARMFLOW_LOCK_OWNER:-claude-lead}"
        echo "branch: $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
        echo "task: $task"
        echo "start_utc: $(date -u +%Y-%m-%dT%H:%MZ)"
        echo "expected_end_utc: $(date -u -d "+$minutes min" +%Y-%m-%dT%H:%MZ 2>/dev/null || echo "+${minutes}min")"
      } > "$dir/owner.txt"
      token="$(date +%s%N)-$$-${RANDOM}"
      echo "token: $token" >> "$dir/owner.txt"
      mkdir -p "$tokdir" && echo "$token" > "$tokdir/$name.token"
      echo "lock $name acquired ($dir)"
    else
      echo "lock $name is held ($(age_min) min):" >&2; cat "$dir/owner.txt" >&2 2>/dev/null || true
      exit 1
    fi ;;
  release)
    if [ -d "$dir" ] && [ "${3:-}" != "--force" ]; then
      held="$(sed -n 's/^token: //p' "$dir/owner.txt" 2>/dev/null)"
      mine="$(cat "$tokdir/$name.token" 2>/dev/null)"
      if [ -n "$held" ] && [ "$held" != "$mine" ]; then
        echo "lock $name is held by someone else; not released (use --force only for a stale lock):" >&2
        cat "$dir/owner.txt" >&2; exit 1
      fi
    fi
    rm -rf "$dir"; rm -f "$tokdir/$name.token"; echo "lock $name released" ;;
  status)
    if [ -d "$dir" ]; then echo "held ($(age_min) min)"; cat "$dir/owner.txt" 2>/dev/null || true; else echo "free"; fi ;;
  age)
    if [ -d "$dir" ]; then age_min; else echo -1; fi ;;
  *) echo "unknown command $cmd" >&2; exit 2 ;;
esac
