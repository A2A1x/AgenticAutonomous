# AgenticAutonomous

Decision points in autos, tested on Spectrum 3847's 2024 CRESCENDO robot, Ultraviolet.

**Agentic Rush** is a PathPlanner auto with decision nodes:

1. The robot starts on the source side, shoots its preload on the move and rushes under the
   stage.
2. At each node, an agent picks which centre note to take from what the robot sees: notes
   gone, opponents that will get there first, robots in the way, notes a partner should
   take, and loose notes it can steal.

Everything runs in simulation: maple-sim physics, with three other simulated Ultraviolets
running the team's real 2024 autos.

**Write-up:** [a2a1x.github.io/AgenticAutonomous](https://a2a1x.github.io/AgenticAutonomous/)
covers where the idea came from, the 2026 tests and a replay of every simulated run. The page
is `docs/index.html`.

The robot code and its history come from
[Spectrum3847/2024-Ultraviolet](https://github.com/Spectrum3847/2024-Ultraviolet), Spectrum's
2024 competition software, upgraded here to WPILib 2026.

## What you need

- **WPILib 2026** (GradleRIO 2026.2.1). Point `JAVA_HOME` at its JDK before running Gradle,
  or the build fails on Java 8:

  ```bash
  export JAVA_HOME=/c/Users/Public/wpilib/2026/jdk
  ```

- **Python 3** for the log tools in `tools/`.
- **AdvantageScope** and **Elastic** to watch the sim.
- **Flutter** only if you want to edit the decision autos (see
  [Editing autos](#editing-autos)).

## Run the sim

```bash
./gradlew simulateJava
```

1. **Load the dashboard.** In Elastic, open `elastic-layout.json`. The **Sim** tab has:
   - the auto chooser (pick **Agentic Rush**);
   - a chooser per sim robot: Off, Random, or a specific auto;
   - C1–C5 toggles for which centre notes spawn;
   - **Off limits** toggles for notes the agent should leave to a partner.
2. **Start auto.** Enable Autonomous in the sim GUI. While disabled, the robot moves itself
   to the chosen auto's start pose.
3. **Watch it in AdvantageScope.**
   - Live, connect to the sim and open `advantagescope-layouts/Agentic Rush (live).json`.
   - Afterwards, open a log from `logs/` with `Agentic Rush (log).json`.
   - To see the robot model instead of a box, set AdvantageScope's custom assets folder to
     `advantagescope-custom-assets`.

Useful log keys, under `/RealOutputs/`:

| Key | What it holds |
| --- | --- |
| `Agent/Input/*` | What the camera sees this frame |
| `Agent/Map/*` | What the agent believes: each centre spot's state, loose notes, robots and their headings |
| `Agent/Output/Considered` | Every option the agent scored, with its reasons |
| `Agent/Output/Choice`, `Agent/Output/Outcome` | What it picked |
| `Agent/Output/StealWaypoint` | Where a steal moved the intake point |
| `Sim/Robot/TruePose` | Where the robot really is, as opposed to `Odometry/Robot` |
| `Sim/Field/Notes`, `Sim/Field/NotesInFlight` | Notes on the field and in the air |
| `Sim/Others/*` | The other robots: poses, autos, paths, notes launched and scored |

## Headless runs

Scripted runs pick their own auto, run it some number of times and can quit when done:

```bash
UV_SIM_AUTO="Agentic Rush" UV_SIM_AUTO_RUNS=6 UV_SIM_ROBOTS=1 UV_SIM_EXIT=1 ./gradlew simulateJava -PnoSimGui
```

Each run is 1 s disabled, then 15 s of auto. Every run lands in one `logs/akit_*.wpilog`.

| Variable | Does |
| --- | --- |
| `UV_SIM_AUTO` | The auto to run, by its PathPlanner name. Setting it is what makes a run scripted. |
| `UV_SIM_AUTO_RUNS` | How many times to run it. Default 1. |
| `UV_SIM_AUTO_SECONDS` | Seconds of auto per run, to stop early. Default 15. |
| `UV_SIM_DISABLED_SECONDS` | Seconds disabled before each run. Default 1. |
| `UV_SIM_EXIT` | `1` quits the sim after the last run. |
| `UV_SIM_ROBOTS` | Which sim robots take part: `1` for all three, or names such as `Red source` or `Blue center,Red source`. Off by default in scripted runs. |
| `UV_SIM_COMBOS` | `1` makes run *n* play combination *n* of the robots' random picks instead of a random one. 18 runs cover every combination. |
| `UV_SIM_SEED` | Seed for the robots' random picks. Default 1. |
| `UV_SIM_VISION` | `0` turns off the simulated AprilTag fixes, so you see raw odometry drift. |
| `UV_SIM_LOOSE_NOTE` | `x,y` in field metres: one extra loose note, to test steals. |
| `UV_AGENT` | `0` leaves the agent out, so every node takes its default option. That's the scripted baseline. |
| `UV_AGENT_OFF_LIMITS` | Notes the agent must leave alone, e.g. `C1,C2`. The headless version of the dashboard toggles. |

The sim robots' random picks:

| Robot | Picks from |
| --- | --- |
| Blue center | Front 6.5, Front Alt 6, Front Rush 12 |
| Red center | Front 6.5, Front Alt 6, Front Rush 12 |
| Red source | Madtown, Source 4 |

There's no blue source robot: 3847 is that robot.

### Comparing the agent with the script

Run the same matchups twice, once as is and once with `UV_AGENT=0`, then:

```bash
python tools/agent_bench.py <agent.wpilog> <script.wpilog>
```

This prints notes launched per run and the mean for each.

`tools/extract_runs.py` packs an all-combinations agent sweep and a matching script sweep into
replay data. Each run gets every robot's true path, the notes on the field and the agent's
reasoning at each decision. `sim-results/agentic-rush-combos.json` is that data for the 18
combinations. The docstring has the exact commands.

`tools/wpilog.py` is the small `.wpilog` reader both tools use. `read(path)` returns
`{key: [(time, value), ...]}`, and `at(series, t)` gives the value at a time.

## Where things live

| Path | What |
| --- | --- |
| `src/main/java/frc/robot/agent/RushAgent.java` | The deciders for Agentic Rush's nodes: how each option is scored, and the path edit that turns an option into a steal |
| `src/main/java/frc/robot/agent/CenterLineMap.java` | What the agent knows about the centre line, built from camera frames and its own pickups |
| `src/main/java/frc/robot/agent/Detections.java` | One camera frame: notes and robots seen, and what the camera covers |
| `src/main/java/frc/robot/sim/RobotSim.java` | maple-sim arena, intake, feeder and launcher sims, aimed shots and spits, simulated vision fixes |
| `src/main/java/frc/robot/sim/SimRobots.java` | The three other Ultraviolets, the autos they run and stand-ins for their named commands |
| `src/main/java/frc/robot/sim/SimCamera.java` | The sim's 360° note and robot camera |
| `src/main/java/frc/robot/sim/SimScript.java` | Headless runs |
| `src/main/java/frc/robot/auton/` | Named commands, the auto chooser and PathPlanner setup |
| `src/main/java/frc/spectrumLib/swerve/MapleSimDrivetrain.java` | The swerve drivetrain in maple-sim |
| `src/main/deploy/pathplanner/` | Paths and autos. Agentic Rush's are in the **Agentic** folder; the 2024 competition ones are in **Comp**. |
| `pathplanner-fork/` | PathPlanner v2026.1.2 with decision nodes and path edits, app and library, tracked as plain files |
| `libs/maven/` | The fork's library, published as `PathplannerLib-java:2026.1.2-decisions` and used by `build.gradle` |

## Editing autos

Agentic Rush's `.auto` file has `"decision"` nodes. The stock PathPlanner app doesn't know
about them, so edit it with the fork's app:

```bash
cd pathplanner-fork
flutter run -d windows
```

Each decision node has:

- a **name**, which the robot code registers a decider under;
- a **timeout** and a **default option**, used if the decider never answers;
- **options**, each a list of commands.

A `decide:<name>` event marker on the path before a node locks the choice in while the robot
is still driving.

## Changing the PathPlanner library

The robot builds against the jar in `libs/maven`, not against `pathplanner-fork` directly.
After changing `pathplanner-fork/pathplannerlib`, republish it. `libs/maven/README.md` has the
command and the decision-node API.

## Gotchas

- **Formatting:** `./gradlew build` runs Spotless, which uses google-java-format 1.7. It can't
  parse newer Java syntax: no `instanceof` patterns, switch expressions or records. Run
  `./gradlew spotlessApply` to fix formatting.
- **Dependency pins:**
  - maple-sim is pinned to 0.4.0-beta, because the version the published vendordep names
    isn't in its repo.
  - Don't add the stock PathplannerLib vendordep back. Gradle ranks `2026.1.2` above
    `2026.1.2-decisions` and quietly uses upstream, so the decision nodes do nothing.
- **Motor sim:** Phoenix's sim models every TalonFX as a Kraken X60, so the maple-sim motor
  models use the Kraken X60 FOC curve.
- **Git Bash:** it rewrites arguments that start with `/` as Windows paths. Prefix a command
  with `MSYS_NO_PATHCONV=1` when you pass it log keys like `/RealOutputs/...`.
