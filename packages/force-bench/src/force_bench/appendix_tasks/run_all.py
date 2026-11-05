"""Run the benchmark tasks and collect their verdicts.

    python -m force_bench.appendix_tasks.run_all                 # all, 3 at a time
    python -m force_bench.appendix_tasks.run_all -j 1 plug_in_ethernet turn_dial_to_target
    python -m force_bench.appendix_tasks.run_all --summary       # only re-collect

Each row runs in its OWN process (a derived task patches module constants
of the scene it builds on; a fresh interpreter guarantees nothing leaks
between rows).  Logs go to outputs/appendix_tasks/_logs/<name>.log;
outputs/appendix_tasks/summary.json collects every appendix_qa.json.  A
task without a scripted policy (``scripted=False``, a scene only) is not
run.  Sims share one GPU; bare rollouts need ~1-2 GB each, the CPU-bound
QA is what limits concurrency.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from force_bench.appendix_tasks.common import APPENDIX_OUT
from force_bench.appendix_tasks.registry import APPENDIX, BY_NAME


def collect() -> dict:
    rows = {}
    for s in APPENDIX:
        if not s.scripted:
            rows[s.name] = {"appendix_task": s.appendix_task, "source": s.source,
                            "status": "scene only (no scripted policy)"}
            continue
        p = APPENDIX_OUT / s.name / "appendix_qa.json"
        if not p.exists():
            rows[s.name] = {"appendix_task": s.appendix_task, "source": s.source,
                            "status": "not run"}
            continue
        d = json.loads(p.read_text())
        rows[s.name] = {
            "appendix_task": s.appendix_task, "source": s.source,
            "pass": d["pass"],
            "failed": [k for k, c in d["criteria"].items() if not c.get("pass")]
            + [k for k, ok in d.get("physics_integrity", {}).items() if not ok],
            "video": str(APPENDIX_OUT / s.name / "rollout.mp4"),
        }
    out = APPENDIX_OUT / "summary.json"
    out.write_text(json.dumps(rows, indent=1))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*")
    ap.add_argument("-j", "--jobs", type=int, default=3)
    ap.add_argument("--summary", action="store_true")
    a = ap.parse_args(argv)
    if not a.summary:
        todo = [BY_NAME[n] for n in a.names] if a.names else list(APPENDIX)
        todo = [s for s in todo if s.scripted]
        logs = APPENDIX_OUT / "_logs"
        logs.mkdir(parents=True, exist_ok=True)
        running: list[tuple[subprocess.Popen, str, float]] = []
        while todo or running:
            while todo and len(running) < a.jobs:
                s = todo.pop(0)
                f = open(logs / f"{s.name}.log", "w")
                p = subprocess.Popen([sys.executable, "-u", "-m", s.dotted, *s.argv],
                                     stdout=f, stderr=subprocess.STDOUT)
                running.append((p, s.name, time.time()))
                print(f"started {s.name}")
            for p, n, t0 in list(running):
                if p.poll() is not None:
                    running.remove((p, n, t0))
                    print(f"finished {n} (rc {p.returncode}, {time.time() - t0:.0f} s)")
            time.sleep(2)
    rows = collect()
    w = max(len(n) for n in rows)
    for n, r in rows.items():
        status = r.get("status") or ("PASS" if r["pass"] else "FAIL " + ",".join(r["failed"]))
        print(f"{n:<{w}}  {status}")


if __name__ == "__main__":
    main()
