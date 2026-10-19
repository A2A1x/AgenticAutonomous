"""Compares Agentic Rush runs with the agent and on its defaults, run by run.

    python tools/agent_bench.py <agent.wpilog> <defaults.wpilog>

Each log holds the runs of one UV_SIM_AUTO_RUNS sim. Runs with the same number share a seed,
so the other robots run the same autos in both.
"""

import sys

from wpilog import at, read


def runs(path):
    log = read(path)
    en = log["/DriverStation/Enabled"]
    starts = [t for (t, v), (_, p) in zip(en[1:], en) if v and not p]
    k = lambda n: log.get("/RealOutputs/" + n, [])
    out = []
    for t0 in starts:
        choices = [v for t, v in k("Agent/Output/Outcome") if t0 <= t <= t0 + 15]
        out.append({
            "others": at(k("Sim/Others/Autos"), t0 + 0.5),
            "launched": at(k("Sim/Robot/NotesLaunched"), t0 + 15) or 0,
            "choices": choices,
        })
    return out


if __name__ == "__main__":
    agent, defaults = runs(sys.argv[1]), runs(sys.argv[2])
    total_a = total_d = 0
    for i, (a, d) in enumerate(zip(agent, defaults)):
        total_a += a["launched"]
        total_d += d["launched"]
        print(f"run {i + 1}: others {a['others']}")
        print(f"   agent {a['launched']} notes  {a['choices']}")
        print(f"   defaults {d['launched']} notes")
    n = min(len(agent), len(defaults))
    print(f"mean notes in 15 s: agent {total_a / n:.2f}, defaults {total_d / n:.2f}")
