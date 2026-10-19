package frc.robot.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Robot;
import frc.robot.auton.config.AutonConfig;
import frc.spectrumLib.swerve.MapleSimDrivetrain;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.littletonrobotics.junction.Logger;

/**
 * The other robots on the field during auto: a blue partner in the middle running a random Front
 * auto, and on red one in the middle (random Front auto) and one on the source side (Madtown or
 * Source 4). 3847 is blue's source-side robot, so there's no blue source robot here. A dashboard
 * chooser per robot (Sim Robots/...) picks a specific auto instead, or Off to leave it off the
 * field. They are Ultraviolets running Ultraviolet's own autos, so they race us for the same centre
 * notes.
 */
public final class SimRobots {
    private static final String[] CENTER_AUTOS = {
        "Front 6.5",
        "Front Alt 6",
        "Front 43",
        "Front Rush 12",
        "Front Rush 23",
        "Front Rush 13",
        "Front Rush 3"
    };

    /**
     * The centre robots' random pick: the Front autos that stay out from under the stage, which
     * Agentic Rush (3847) works through. Front 43 and Front Rush 3, 13 and 23 drive under it.
     */
    private static final String[] CENTER_RANDOM = {"Front 6.5", "Front Alt 6", "Front Rush 12"};

    private static final String SOURCE_AUTO = "Source 4";

    /**
     * Madtown is 3847's own rush, from the red side. It reaches C3 before Agentic Rush and spits
     * its preload, so the agent has a note to dodge and one to steal.
     */
    private static final String RED_SOURCE_AUTO = "Madtown";

    /** Red source's random pick. */
    private static final String[] SOURCE_RANDOM = {RED_SOURCE_AUTO, SOURCE_AUTO};

    /** Chooser options besides the autos: the role's default pick, and not on the field. */
    private static final String RANDOM = "Random", OFF = "Off";

    /** How long a {@code launch} keeps the feeder running, as AutonCommands.launch does. */
    private static final double LAUNCH_S = 0.35;

    private static final long SEED = parseOr(System.getenv("UV_SIM_SEED"), 1);

    private static final List<SimRobot> robots = new ArrayList<>();

    /** True while a sim robot's path follower runs, so PathPlanner's logging can skip it. */
    private static boolean muted;

    private static int run;

    private SimRobots() {}

    /** Whether PathPlanner's logging callbacks are hearing from a sim robot right now. */
    public static boolean isMuted() {
        return muted;
    }

    /**
     * Builds our centre partner and two opponents, parked off the field. 3847 is blue's source
     * robot.
     */
    public static void init() {
        // ponytail: maple-sim has one battery for every robot, so five drivetrains brown it out.
        // Hand back the other robots' draw so the battery only sees ours; drop this if maple-sim
        // gets per-robot batteries.
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(-otherRobotsCurrent()));
        robots.add(new SimRobot("Blue center", false, new Pose2d(-2, 1, Rotation2d.kZero)));
        robots.add(new SimRobot("Red center", true, new Pose2d(-2, 5, Rotation2d.kZero)));
        robots.add(new SimRobot("Red source", true, new Pose2d(-2, 7, Rotation2d.kZero)));
    }

    private static double otherRobotsCurrent() {
        double amps = 0;
        for (SimRobot r : robots) {
            for (SwerveModuleSimulation m : r.drive.getDriveTrainSimulation().getModules()) {
                amps += m.getDriveMotorSupplyCurrent().in(Amps);
                amps += m.getSteerMotorSupplyCurrent().in(Amps);
            }
        }
        return amps;
    }

    /** Whether a scripted run puts this robot on the field, per UV_SIM_ROBOTS. */
    private static boolean scripted(String name) {
        String on = System.getenv("UV_SIM_ROBOTS");
        return on != null && ("1".equals(on) || List.of(on.split(",")).contains(name));
    }

    /**
     * Places every robot on its start and runs the auto its chooser picks. Call at the start of
     * auto.
     */
    public static void start() {
        Random random = new Random(SEED * 1000 + run++);
        String[] autos = new String[robots.size()];
        for (int i = 0; i < robots.size(); i++) {
            SimRobot r = robots.get(i);
            // Scripted runs don't take whatever a dashboard last left selected: robots are off
            // unless UV_SIM_ROBOTS is 1 (all of them) or lists their names ("Red source,...").
            String choice =
                    SimScript.auto() == null
                            ? r.chooser.getSelected()
                            : scripted(r.name) ? RANDOM : OFF;
            if (OFF.equals(choice)) {
                autos[i] = OFF;
                r.park();
                continue;
            }
            if (choice == null || RANDOM.equals(choice)) {
                String[] pool = r.name.endsWith("center") ? CENTER_RANDOM : SOURCE_RANDOM;
                choice = pool[random.nextInt(pool.length)];
            }
            autos[i] = choice;
            r.run(choice);
        }
        Logger.recordOutput("Sim/Others/Autos", autos);
        Logger.recordOutput("Sim/Others/Seed", SEED * 1000 + run - 1);
    }

    /** Stops every robot and parks it. Call when the robot disables. */
    public static void stop() {
        robots.forEach(SimRobot::park);
    }

    /** Where the robots on the field really are; parked ones are left out. */
    public static List<Pose2d> onField() {
        List<Pose2d> out = new ArrayList<>();
        for (SimRobot r : robots) {
            Pose2d p = r.drive.getActualPoseInSimulationWorld();
            if (p.getX() > 0) {
                out.add(p);
            }
        }
        return out;
    }

    /** Whether each robot in {@link #onField()} is red, in the same order. */
    public static List<Boolean> onFieldRed() {
        List<Boolean> out = new ArrayList<>();
        for (SimRobot r : robots) {
            if (r.drive.getActualPoseInSimulationWorld().getX() > 0) {
                out.add(r.red);
            }
        }
        return out;
    }

    /** Steps the robots' drivetrains. Call before the arena ticks. */
    public static void periodic() {
        Pose2d[] poses = new Pose2d[robots.size()];
        long[] launched = new long[robots.size()];
        long[] scored = new long[robots.size()];
        for (int i = 0; i < robots.size(); i++) {
            if (robots.get(i).auto == null || !robots.get(i).auto.isScheduled()) {
                robots.get(i).holdStill();
            }
            robots.get(i).drive.periodic();
            poses[i] = robots.get(i).drive.getActualPoseInSimulationWorld();
            launched[i] = robots.get(i).launched;
            scored[i] = robots.get(i).scored;
        }
        Logger.recordOutput("Sim/Others/Poses", poses);
        for (SimRobot r : robots) {
            Logger.recordOutput("Sim/Others/" + r.key + "/Path", r.path);
            Robot.swerve.getField().getObject(r.name + " path").setPoses(r.path);
        }
        Robot.swerve.getField().getObject("Sim Robots").setPoses(SimRobots.onField());
        Logger.recordOutput("Sim/Others/Launched", launched);
        Logger.recordOutput("Sim/Others/Scored", scored);
    }

    private static class SimRobot extends SubsystemBase {
        final String name;
        final boolean red;
        final Pose2d parked;
        final SelfControlledSwerveDriveSimulation drive;
        final IntakeSimulation intake;
        final PPHolonomicDriveController controller =
                new PPHolonomicDriveController(
                        new PIDConstants(AutonConfig.kTranslationP, 0, 0),
                        new PIDConstants(AutonConfig.kRotationP, 0, 0));
        /** Stands in for the feeder, so feeder actions interrupt each other. */
        final SubsystemBase feeder = new SubsystemBase() {};

        /** What this robot runs: Random (its role's pick), an auto, or Off. */
        final SendableChooser<String> chooser = new SendableChooser<>();

        Command auto;
        int launched, scored;

        /** Its log key, e.g. BlueCenter. */
        final String key;

        /** The path it's following now, flipped for red; empty when parked. */
        Pose2d[] path = new Pose2d[0];

        SimRobot(String name, boolean red, Pose2d parked) {
            this.name = name;
            String[] words = name.split(" ");
            StringBuilder k = new StringBuilder();
            for (String w : words) {
                k.append(Character.toUpperCase(w.charAt(0))).append(w.substring(1));
            }
            this.key = k.toString();
            this.red = red;
            this.parked = parked;
            SwerveDriveSimulation body =
                    new SwerveDriveSimulation(
                            MapleSimDrivetrain.driveTrainConfig(Robot.swerve.config), parked);
            SimulatedArena.getInstance().addDriveTrainSimulation(body);
            drive = new SelfControlledSwerveDriveSimulation(body);
            intake =
                    IntakeSimulation.InTheFrameIntake(
                            "Note", body, Meters.of(0.55), IntakeSimulation.IntakeSide.BACK, 1);
            intake.register(SimulatedArena.getInstance());

            chooser.setDefaultOption(OFF, OFF); // off until they're wanted
            chooser.addOption(RANDOM, RANDOM);
            for (String a : CENTER_AUTOS) {
                chooser.addOption(a, a);
            }
            chooser.addOption(SOURCE_AUTO, SOURCE_AUTO);
            chooser.addOption(RED_SOURCE_AUTO, RED_SOURCE_AUTO);
            SmartDashboard.putData("Sim Robots/" + name, chooser);
        }

        void run(String autoName) {
            Pose2d start;
            try {
                JSONObject json =
                        (JSONObject)
                                new JSONParser().parse(new FileReader(autoFile(autoName).toFile()));
                auto = build((JSONObject) json.get("command"));
                start = startingPose(autoName);
            } catch (Exception e) {
                DriverStation.reportError(name + " can't load " + autoName + ": " + e, false);
                return;
            }
            launched = 0;
            scored = 0;
            intake.setGamePiecesCount(1);
            intake.startIntake();
            drive.setSimulationWorldPose(start);
            drive.resetOdometry(start);
            CommandScheduler.getInstance().schedule(auto);
        }

        void park() {
            if (auto != null) {
                auto.cancel();
            }
            intake.stopIntake();
            holdStill();
            drive.setSimulationWorldPose(parked);
            path = new Pose2d[0];
        }

        /**
         * Stopped, with every module turning to straight ahead, as at power-on. Call every loop
         * while no auto drives it: maple-sim sets a module's voltages only when it's given a state,
         * so one call would leave the steer motor spinning at that voltage for as long as the robot
         * sits (10,000 degrees over a 12 s disable), and the next auto would crawl while it
         * unwinds.
         */
        void holdStill() {
            SwerveModuleState straight = new SwerveModuleState(0, Rotation2d.kZero);
            drive.runSwerveStates(new SwerveModuleState[] {straight, straight, straight, straight});
        }

        private Pose2d startingPose(String autoName) {
            PathPlannerPath first = firstPath(autoName);
            Pose2d start =
                    first.getStartingHolonomicPose()
                            .orElse(
                                    new Pose2d(
                                            first.getWaypoints().get(0).anchor(),
                                            first.getIdealStartingState().rotation()));
            return red ? FlippingUtil.flipFieldPose(start) : start;
        }

        private PathPlannerPath firstPath(String autoName) {
            try {
                JSONObject json =
                        (JSONObject)
                                new JSONParser().parse(new FileReader(autoFile(autoName).toFile()));
                return PathPlannerPath.fromPathFile(
                        firstPathName((JSONObject) json.get("command")));
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        private static String firstPathName(JSONObject node) {
            JSONObject data = (JSONObject) node.get("data");
            if ("path".equals(node.get("type"))) {
                return (String) data.get("pathName");
            }
            if (data.get("commands") instanceof JSONArray) {
                for (Object c : (JSONArray) data.get("commands")) {
                    String found = firstPathName((JSONObject) c);
                    if (found != null) {
                        return found;
                    }
                }
            }
            return null;
        }

        /** The auto's command tree, with named commands swapped for this robot's stand-ins. */
        private Command build(JSONObject node) throws Exception {
            JSONObject data = (JSONObject) node.get("data");
            String type = (String) node.get("type");
            if (type.equals("sequential")
                    || type.equals("parallel")
                    || type.equals("race")
                    || type.equals("deadline")) {
                List<Command> children = new ArrayList<>();
                for (Object c : (JSONArray) data.get("commands")) {
                    children.add(build((JSONObject) c));
                }
                Command[] all = children.toArray(new Command[0]);
                if (type.equals("sequential")) {
                    return Commands.sequence(all);
                } else if (type.equals("parallel")) {
                    return Commands.parallel(all);
                } else if (type.equals("race")) {
                    return Commands.race(all);
                }
                return all.length == 0
                        ? Commands.none()
                        : Commands.deadline(
                                all[0], children.subList(1, all.length).toArray(new Command[0]));
            } else if (type.equals("wait")) {
                return Commands.waitSeconds(((Number) data.get("waitTime")).doubleValue());
            } else if (type.equals("path")) {
                return follow(loadPath((String) data.get("pathName")));
            } else if (type.equals("named")) {
                return named((String) data.get("name"));
            }
            return Commands.none();
        }

        /**
         * This robot's stand-in for one of Ultraviolet's named commands. Each feeder action needs
         * the robot's feeder, so a new one interrupts the last, as on the real robot.
         */
        private Command named(String name) {
            if (name == null || name.startsWith("launchReady") || name.equals("spitReady")) {
                return Commands.none();
            }
            if (name.equals("intakeFeed")) {
                return Commands.run(this::shoot, feeder); // shoots each note straight through
            }
            if (name.startsWith("spitReady")) {
                // spitReady2 and spitReady3 run a spit too: each note goes straight through.
                return Commands.run(this::spit, feeder);
            }
            if (name.startsWith("spit")) {
                return Commands.runOnce(this::spit, feeder).andThen(Commands.waitSeconds(LAUNCH_S));
            }
            if (name.startsWith("launch")) {
                // No feeder requirement: a launch marker that fires in the same loop as an
                // intakeFeed one (Front 6's short first path) would otherwise cancel the feed.
                return Commands.runOnce(this::shoot).andThen(Commands.waitSeconds(LAUNCH_S));
            }
            if (name.equals("smartIntake") || name.equals("intake")) {
                return Commands.runOnce(() -> {}, feeder); // stop feeding through; hold the next
            }
            return Commands.none();
        }

        /**
         * Drops the held note on the field ahead, where a camera can see it and a robot can take
         * it.
         */
        private void spit() {
            if (intake.obtainGamePieceFromIntake()) {
                RobotSim.spit(drive.getActualPoseInSimulationWorld());
            }
        }

        private void shoot() {
            if (intake.obtainGamePieceFromIntake()) {
                launched++;
                RobotSim.aimedShot(drive.getActualPoseInSimulationWorld(), red, () -> scored++);
            }
        }

        private Command follow(PathPlannerPath path) {
            Command follow =
                    new FollowPathCommand(
                            path,
                            drive::getActualPoseInSimulationWorld,
                            drive::getActualSpeedsRobotRelative,
                            (speeds, feedforwards) ->
                                    drive.runChassisSpeeds(
                                            speeds, new Translation2d(), false, false),
                            controller,
                            AutonConfig.robotConfig(),
                            () -> red,
                            this);
            // PathPlanner's logging is global; keep these robots out of our PathPlanner/ logs.
            return new WrapperCommand(follow) {
                @Override
                public void initialize() {
                    muted = true;
                    super.initialize();
                    SimRobot.this.path =
                            (red ? path.flipPath() : path).getPathPoses().toArray(new Pose2d[0]);
                    muted = false;
                }

                @Override
                public void execute() {
                    muted = true;
                    super.execute();
                    muted = false;
                }

                @Override
                public void end(boolean interrupted) {
                    muted = true;
                    super.end(interrupted);
                    muted = false;
                }
            };
        }

        /**
         * The path with its event markers rebuilt from the file, so they run this robot's stand-ins
         * instead of our robot's named commands.
         */
        private PathPlannerPath loadPath(String pathName) throws Exception {
            PathPlannerPath p = PathPlannerPath.fromPathFile(pathName);
            JSONObject json =
                    (JSONObject)
                            new JSONParser()
                                    .parse(
                                            new FileReader(
                                                    Filesystem.getDeployDirectory()
                                                            .toPath()
                                                            .resolve(
                                                                    "pathplanner/paths/"
                                                                            + pathName
                                                                            + ".path")
                                                            .toFile()));
            List<EventMarker> markers = new ArrayList<>();
            for (Object o : (JSONArray) json.get("eventMarkers")) {
                JSONObject m = (JSONObject) o;
                Object end = m.get("endWaypointRelativePos");
                Command command =
                        m.get("command") == null
                                ? Commands.none()
                                : build((JSONObject) m.get("command"));
                double start = ((Number) m.get("waypointRelativePos")).doubleValue();
                markers.add(
                        end == null
                                ? new EventMarker((String) m.get("name"), start, command)
                                : new EventMarker(
                                        (String) m.get("name"),
                                        start,
                                        ((Number) end).doubleValue(),
                                        command));
            }
            return new PathPlannerPath(
                    p.getWaypoints(),
                    p.getRotationTargets(),
                    p.getPointTowardsZones(),
                    p.getConstraintZones(),
                    markers,
                    p.getGlobalConstraints(),
                    p.getIdealStartingState(),
                    p.getGoalEndState(),
                    p.isReversed());
        }

        private static Path autoFile(String autoName) {
            return Filesystem.getDeployDirectory()
                    .toPath()
                    .resolve("pathplanner/autos/" + autoName + ".auto");
        }
    }

    private static long parseOr(String s, long fallback) {
        try {
            return s == null ? fallback : Long.parseLong(s.trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }
}
