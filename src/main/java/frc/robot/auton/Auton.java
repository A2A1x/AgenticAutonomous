package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.agent.RushAgent;
import frc.robot.auton.config.AutonConfig;
import frc.robot.mechanisms.amptrap.AmpTrapCommands;
import frc.robot.mechanisms.feeder.FeederCommands;
import frc.robot.mechanisms.intake.IntakeCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.mechanisms.pivot.PivotCommands;
import frc.robot.sim.SimScript;
import java.util.HashMap;
import java.util.Map;

/**
 * Autonomous setup: the named commands PathPlanner autos call, the deciders behind their decision
 * nodes, and the auto chooser. Named commands and deciders must be registered before any auto is
 * built.
 */
public class Auton extends SubsystemBase {
    public static final SendableChooser<Command> autonChooser = new SendableChooser<>();
    private static final Map<String, Command> scriptedAutos = new HashMap<>();

    /* State the auto commands share (see AutonCommands) */
    public static boolean trackNote = false;
    public static boolean trackSpeaker = false;
    public static boolean noteIntaked = false;
    public static boolean intakeCheck = false;

    public Auton() {
        setupNamedCommands();
        RushAgent.register(); // Agentic Rush's deciders and their decide: commands
        AutonConfig.configureAutoBuilder();
        setupSelectors();

        RobotTelemetry.print("Auton Subsystem Initialized: ");
    }

    /** The autos on the dashboard's Auto Chooser. */
    public static void setupSelectors() {
        autonChooser.addOption("Agentic Rush", new PathPlannerAuto("Agentic Rush"));
        autonChooser.addOption("Do Nothing", Commands.print("Do Nothing Auto ran"));

        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    /** Every named command the autos call. The names are what the .auto and .path files use. */
    public static void setupNamedCommands() {
        /* Aiming */
        NamedCommands.registerCommand("alignToSpeaker", AutonCommands.trackSpeaker());
        NamedCommands.registerCommand("alignToNote", AutonCommands.trackNote());
        NamedCommands.registerCommand("stopTracking", AutonCommands.stopTracking());

        /* Getting ready to launch: pivot and launcher to a shot's setpoints */
        NamedCommands.registerCommand("launchReadySubwoofer", AutonCommands.launchReadySubwoofer());
        NamedCommands.registerCommand("launchReadyPreload", AutonCommands.launchReadyPreload());
        NamedCommands.registerCommand("launchReadyPreload2", AutonCommands.launchReadyPreload2());
        NamedCommands.registerCommand("launchReady1", AutonCommands.launchReady1());
        NamedCommands.registerCommand("launchReady2", AutonCommands.launchReady2());
        NamedCommands.registerCommand("launchReady3", AutonCommands.launchReady3());
        NamedCommands.registerCommand("launchReady4", AutonCommands.launchReady4());
        NamedCommands.registerCommand("launchReady5", AutonCommands.launchReady5());
        NamedCommands.registerCommand("launchReady6", AutonCommands.launchReady6());
        NamedCommands.registerCommand("launchReady7", AutonCommands.launchReady7());
        NamedCommands.registerCommand("launchReady8", AutonCommands.launchReady8());
        NamedCommands.registerCommand("launchReady9", AutonCommands.launchReady9());
        NamedCommands.registerCommand("launchReady10", AutonCommands.launchReady10());
        NamedCommands.registerCommand("launchReady11", AutonCommands.launchReady11());
        NamedCommands.registerCommand("launchReady12", AutonCommands.launchReady12());
        NamedCommands.registerCommand("launchReady13", AutonCommands.launchReady13());
        NamedCommands.registerCommand("launchReady14", AutonCommands.launchReady14());
        NamedCommands.registerCommand("launchReady15", AutonCommands.launchReady15());
        NamedCommands.registerCommand("visionLaunchReady", AutonCommands.visionLaunch());

        /* Launching */
        NamedCommands.registerCommand("launch", AutonCommands.launch());
        NamedCommands.registerCommand("launchShort", AutonCommands.launchShort());

        /* Intaking */
        NamedCommands.registerCommand("smartIntake", AutonCommands.intake());
        NamedCommands.registerCommand("intakeFeed", AutonCommands.intakeFeed());
        NamedCommands.registerCommand("intakeCheck", AutonCommands.intakeCheck());
        NamedCommands.registerCommand("stopSmartIntake", AutonCommands.stopFeed());
        // The misspelling is the name autos already use.
        NamedCommands.registerCommand("stophIntakeCheck", AutonCommands.stopIntakeCheck());

        /* Spitting a note out low */
        NamedCommands.registerCommand("spitReady", AutonCommands.spitReady());
        NamedCommands.registerCommand("spitReady2", AutonCommands.spitReady2());
        NamedCommands.registerCommand("spit", AutonCommands.spit());
        NamedCommands.registerCommand("spit2", AutonCommands.spit2());
        NamedCommands.registerCommand("spit4", AutonCommands.spit4());

        /* Pose */
        NamedCommands.registerCommand("resetPose", AutonCommands.resetPoseToVision());

        /* Stopping mechanisms */
        NamedCommands.registerCommand("stopIntake", IntakeCommands.stopMotor());
        NamedCommands.registerCommand("stopFeeder", FeederCommands.stopMotor());
        NamedCommands.registerCommand("stopAmpTrap", AmpTrapCommands.stopMotor());
        NamedCommands.registerCommand("stopLauncher", LauncherCommands.stopMotors());
        NamedCommands.registerCommand("stopPivot", PivotCommands.stopMotor());
    }

    /** How far odometry may be from the auto's start before disable reseeds it, m and deg. */
    private static final double SEEDED_M = 0.05, SEEDED_DEG = 2;

    /**
     * Puts odometry (and the robot, in sim) at the chosen auto's start, flipped for red, whenever
     * it isn't already there. Call while disabled.
     */
    public static void seedStartPose() {
        Command auto = getAutonomousCommand();
        if (!(auto instanceof PathPlannerAuto)) {
            return;
        }
        Pose2d start = ((PathPlannerAuto) auto).getStartingPose();
        if (start == null) {
            return;
        }
        if (AutoBuilder.shouldFlip()) {
            start = FlippingUtil.flipFieldPose(start);
        }
        Pose2d now = Robot.swerve.getPose();
        if (now.getTranslation().getDistance(start.getTranslation()) > SEEDED_M
                || Math.abs(now.getRotation().minus(start.getRotation()).getDegrees())
                        > SEEDED_DEG) {
            Robot.swerve.resetPose(start);
        }
    }

    /** The auto to run: the chooser's, or in a headless sim run, the one it names. */
    public static Command getAutonomousCommand() {
        if (RobotBase.isSimulation() && SimScript.auto() != null) {
            // One instance per auto, reused like the chooser's.
            return scriptedAutos.computeIfAbsent(SimScript.auto(), PathPlannerAuto::new);
        }
        Command auton = autonChooser.getSelected();
        return auton != null ? auton : Commands.print("*** AUTON COMMAND IS NULL ***");
    }
}
