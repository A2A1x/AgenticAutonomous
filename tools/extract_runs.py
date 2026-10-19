"""Packs the all-combinations sim logs into replay data for the artifact's visualizer.

    python tools/extract_runs.py <agent.wpilog> <script.wpilog> <out.json>

Each log holds 18 Agentic Rush runs, one per combination of the other robots' autos, from

    UV_SIM_COMBOS=1 UV_SIM_ROBOTS=1 UV_SIM_AUTO="Agentic Rush" UV_SIM_AUTO_RUNS=18 UV_SIM_EXIT=1 \
        ./gradlew simulateJava -PnoSimGui

once as is and once with UV_AGENT=0 for the script.
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wpilog import at, read  # noqa: E402

FPS = 10
SECONDS = 15.0
# The script takes each node's default option (Agentic Rush.auto).
SCRIPT_CHOICES = ["First Decision: C3", "Decision: C4", "Decision: C2"]


def r2(x):
    return round(x, 2)


def runs(path, mode):
    log = read(path)
    k = lambda n: log.get("/RealOutputs/" + n, [])
    en = log["/DriverStation/Enabled"]
    starts = [t for (t, v), (_, p) in zip(en[1:], en) if v and not p]
    if en and en[0][1]:
        starts.insert(0, en[0][0])
    out = []
    for t0 in starts:
        frames = [t0 + i / FPS for i in range(int(SECONDS * FPS) + 1)]
        autos = at(k("Sim/Others/Autos"), t0 + 0.5)
        us = []
        for t in frames:
            p = at(k("Sim/Robot/TruePose"), t)
            us += [r2(p[0]), r2(p[1]), round(math.degrees(p[2]))]
        others = []
        for i in range(3):
            track = []
            for t in frames:
                p = at(k("Sim/Others/Poses"), t)[i]
                track += [r2(p[0]), r2(p[1]), round(math.degrees(p[2]))] if p[0] > 0 else [None, None, None]
            others.append(track)
        notes, last = [], None
        for f, t in enumerate(frames):
            flat = []
            for n in at(k("Sim/Field/Notes"), t) or []:
                flat += [r2(n[0]), r2(n[1])]
            if flat != last:
                notes.append([f, flat])
                last = flat
        fly = []
        for f, t in enumerate(frames):
            flat = []
            for n in at(k("Sim/Field/NotesInFlight"), t) or []:
                flat += [r2(n[0]), r2(n[1]), r2(n[2])]
            if flat:
                fly.append([f, flat])
        pickups = [r2(t - t0) for t, v in k("Sim/Robot/NoteInRobot") if t0 < t <= t0 + SECONDS and v]
        decisions = []
        if mode == "agent":
            choices = [(t, v) for t, v in k("Agent/Output/Choice") if t0 <= t <= t0 + SECONDS]
            outcomes = [(t, v) for t, v in k("Agent/Output/Outcome") if t0 <= t <= t0 + SECONDS]
            for i, (t, choice) in enumerate(choices):
                decisions.append({
                    "t": r2(t - t0),
                    "choice": choice,
                    "outcome": outcomes[i][1] if i < len(outcomes) else None,
                    "considered": list(at(k("Agent/Output/Considered"), t) or []),
                })
        launched = at(k("Sim/Robot/NotesLaunched"), t0 + SECONDS) or 0
        scored = at(k("Sim/Robot/NotesScored"), t0 + SECONDS) or 0
        out.append({
            "autos": list(autos),
            "mode": mode,
            "us": us,
            "others": others,
            "notes": notes,
            "fly": fly,
            "pickups": pickups,
            "decisions": decisions,
            "script": SCRIPT_CHOICES if mode == "script" else None,
            "launched": int(launched),
            "scored": int(scored),
            "othersLaunched": [int(x) for x in (at(k("Sim/Others/Launched"), t0 + SECONDS) or [0, 0, 0])],
        })
    return out


if __name__ == "__main__":
    agent = runs(sys.argv[1], "agent")
    script = runs(sys.argv[2], "script")
    data = {"fps": FPS, "seconds": SECONDS, "runs": agent + script}
    with open(sys.argv[3], "w", encoding="utf-8") as f:
        json.dump(data, f, separators=(",", ":"))
    for a, s in zip(agent, script):
        assert a["autos"] == s["autos"], (a["autos"], s["autos"])
        print(" / ".join(a["autos"]), "| agent", a["launched"], "launched", a["scored"], "scored | script",
              s["launched"], "launched", s["scored"], "scored |", [d["outcome"] for d in a["decisions"]])
    print(len(agent), "agent runs,", len(script), "script runs")
