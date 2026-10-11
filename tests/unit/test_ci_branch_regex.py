"""scripts/ci.sh recognises task branches, including suffixed ids such as T018b (X-06)."""

import pathlib
import re
import shutil
import subprocess

import pytest

CI = pathlib.Path(__file__).resolve().parents[2] / "scripts" / "ci.sh"


def _task_of(branch: str) -> str:
    line = next(l for l in CI.read_text().splitlines() if '"$BRANCH" =~ ^ws-' in l)
    regex = re.search(r"=~ (\S+) \]\]", line).group(1)
    script = f'BRANCH="$1"; if [[ "$BRANCH" =~ {regex} ]]; then echo "${{BASH_REMATCH[1]}}"; fi'
    return subprocess.run(["bash", "-c", script, "x", branch], capture_output=True, text=True, check=True).stdout.strip()


@pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")
@pytest.mark.parametrize("branch,task", [
    ("ws-b/T012-fcfs-reservations", "T012"),
    ("ws-d/T018b-agent-fix", "T018b"),
    ("ws-f/T100-x", "T100"),
    ("lead/T021-foxglove-view", ""),
    ("ws-g/T012-x", ""),
    ("ws-b/T12-x", ""),
])
def test_task_branch_ids(branch, task):
    assert _task_of(branch) == task
