package frc.robot.sim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.spectrumLib.lasercan.LaserCanUtil;
import frc.spectrumLib.swerve.MapleSimDrivetrain;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * The CRESCENDO field in maple-sim: notes, Ultraviolet's intake and launches.
 *
 * <p>The intake, feeder and launcher get a plain motor model so their commands finish as on the
 * robot. A note launches once the feeder has carried it {@link #LAUNCH_FEED_ROTATIONS} past where
 * it was picked up, and flies into the speaker: launches don't need to be realistic here. The
 * 4-rotation nudge after intaking doesn't count.
 */
public final class RobotSim {
    /** Feeder rotations from pickup to the flywheels. The post-intake nudge is 4. */
    private static final double LAUNCH_FEED_ROTATIONS = 8;

    /** Intake roller speed, rps, that counts as intaking. Intaking is 33. */
    private static final double INTAKE_RUNNING_RPS = 5;

    /** The feeder's LaserCAN id, and what it reads with and without a note, mm. */
    private static final int FEEDER_LASERCAN = 0;

    private static final int NOTE_SEEN_MM = 100;
    private static final int NO_NOTE_MM = 1000;

    private static Arena2024Crescendo arena;
    private static IntakeSimulation intake;
    private static final List<MotorSim> motors = new ArrayList<>();
    private static MotorSim intakeMotor, feederMotor, launcherMotor;

    private static boolean hadNote;
    private static double feederAtPickup;
    private static int notesLaunched, notesScored;

    /** maple-sim's speaker goals, and the tolerance it gives a note arriving at one. */
    private static final Translation3d BLUE_SPEAKER = new Translation3d(0.02, 5.5825, 2.1);

    private static final Translation3d RED_SPEAKER = new Translation3d(16.52, 5.5825, 2.1);
    private static final Translation3d SPEAKER_TOLERANCE = new Translation3d(0.5, 1.2, 0.3);

    /** Where a note leaves the launcher, m, and how far above the line of sight it's aimed. */
    private static final double LAUNCH_HEIGHT = 0.45;

    private static final double AIM_ABOVE_SIGHT_DEG = 20;

    private RobotSim() {}

    /** The centre line notes, C1 (amp side) to C5 (source side). */
    private static final double CENTER_LINE_X = 8.27;

    private static final double[] CENTER_NOTE_Y = {7.46, 5.78, 4.10, 2.43, 0.75};

    private static String centerNoteKey(int i) {
        return "Sim Field/C" + (i + 1);
    }

    /** Makes the CRESCENDO arena current. Call before the drivetrain is built. */
    public static void init() {
        arena = new Arena2024Crescendo();
        SimulatedArena.overrideInstance(arena);
        for (int i = 0; i < CENTER_NOTE_Y.length; i++) {
            SmartDashboard.setDefaultBoolean(centerNoteKey(i), true);
        }
    }

    /** Hooks up the intake and mechanism motors once the subsystems exist. */
    public static void start() {
        MapleSimDrivetrain drive = Robot.swerve.getSimDrivetrain();
        // Ultraviolet picks up notes over its back bumper and holds one.
        intake =
                IntakeSimulation.InTheFrameIntake(
                        "Note",
                        drive.getBody(),
                        Meters.of(0.55),
                        IntakeSimulation.IntakeSide.BACK,
                        1);
        intake.register(arena);
        intakeMotor = motorSim(Robot.intake.getMotor());
        feederMotor = motorSim(Robot.feeder.getMotor());
        launcherMotor = motorSim(Robot.leftLauncher.getMotor());
        motorSim(Robot.rightLauncher.getMotor());
        SimRobots.init();
        resetForAuto();
    }

    /** A fresh field and a preloaded note, as at the start of a match. */
    public static void resetForAuto() {
        arena.resetFieldForAuto();
        // Take off the centre notes switched off on the dashboard (Sim Field/C1..C5).
        for (int i = 0; i < CENTER_NOTE_Y.length; i++) {
            if (SimScript.auto() != null || SmartDashboard.getBoolean(centerNoteKey(i), true)) {
                continue;
            }
            Translation2d spot = new Translation2d(CENTER_LINE_X, CENTER_NOTE_Y[i]);
            for (GamePieceOnFieldSimulation note : arena.gamePiecesOnField()) {
                if (note.getPoseOnField().getTranslation().getDistance(spot) < 0.3) {
                    arena.removeGamePiece(note);
                    break;
                }
            }
        }
        // Test option: one extra loose note, UV_SIM_LOOSE_NOTE="x,y" (field metres).
        String loose = System.getenv("UV_SIM_LOOSE_NOTE");
        if (loose != null) {
            String[] xy = loose.split(",");
            arena.addGamePiece(
                    new CrescendoNoteOnField(
                            new Translation2d(
                                    Double.parseDouble(xy[0].trim()),
                                    Double.parseDouble(xy[1].trim()))));
        }
        intake.setGamePiecesCount(1);
        hadNote = false;
        notesLaunched = 0;
        notesScored = 0;
    }

    /**
     * Stand-in for the Limelights' AprilTag fixes, which see nothing in the sim: every this many
     * loops, the true pose plus noise goes into the pose estimator, as a fix would. Without it,
     * wheel slip drifts odometry ~30 cm over an auto. UV_SIM_VISION=0 turns it off.
     */
    private static final int VISION_LOOPS = 5;

    /** The fixes' noise and how much the estimator trusts them: m, m, rad. */
    private static final double VISION_NOISE_M = 0.03, VISION_NOISE_RAD = Math.toRadians(1);

    private static final boolean VISION = !"0".equals(System.getenv("UV_SIM_VISION"));
    private static final Random visionNoise = new Random(3847);
    private static int loops;

    public static void periodic() {
        for (MotorSim m : motors) {
            m.update();
        }
        if (velocity(intakeMotor) > INTAKE_RUNNING_RPS) {
            intake.startIntake();
        } else {
            intake.stopIntake();
        }
        SimRobots.periodic();
        arena.simulationPeriodic();
        // ponytail: sees tags from anywhere; gate on tags in view if that ever matters
        if (VISION && ++loops % VISION_LOOPS == 0) {
            Pose2d truth = Robot.swerve.getSimDrivetrain().getTruePose();
            Pose2d fix =
                    new Pose2d(
                            truth.getX() + visionNoise.nextGaussian() * VISION_NOISE_M,
                            truth.getY() + visionNoise.nextGaussian() * VISION_NOISE_M,
                            truth.getRotation()
                                    .plus(
                                            Rotation2d.fromRadians(
                                                    visionNoise.nextGaussian()
                                                            * VISION_NOISE_RAD)));
            Robot.swerve.addVisionMeasurement(
                    fix,
                    Timer.getFPGATimestamp(),
                    VecBuilder.fill(VISION_NOISE_M, VISION_NOISE_M, VISION_NOISE_RAD));
            Logger.recordOutput("Sim/Robot/VisionFix", fix);
        }

        boolean hasNote = intake.getGamePiecesAmount() > 0;
        double feeder = position(feederMotor);
        if (hasNote && !hadNote) {
            feederAtPickup = feeder;
        }
        if (hasNote && feeder - feederAtPickup > LAUNCH_FEED_ROTATIONS) {
            intake.obtainGamePieceFromIntake();
            hasNote = false;
            // Proof of concept: every note that reaches the flywheels goes into the speaker.
            notesLaunched++;
            aimedShot(Robot.swerve.getSimDrivetrain().getTruePose(), false, () -> notesScored++);
        }
        hadNote = hasNote;
        LaserCanUtil.simDistanceMm.put(FEEDER_LASERCAN, hasNote ? NOTE_SEEN_MM : NO_NOTE_MM);

        Logger.recordOutput("Sim/Robot/TruePose", Robot.swerve.getSimDrivetrain().getTruePose());
        Pose3d[] notes = arena.getGamePiecesArrayByType("Note");
        Logger.recordOutput("Sim/Field/Notes", notes);
        Robot.swerve
                .getField()
                .getObject("Notes")
                .setPoses(Arrays.stream(notes).map(Pose3d::toPose2d).toArray(Pose2d[]::new));
        Logger.recordOutput(
                "Sim/Field/NotesInFlight",
                arena.gamePieceLaunched().stream()
                        .map(GamePieceProjectile::getPose3d)
                        .toArray(Pose3d[]::new));
        Logger.recordOutput("Sim/Robot/NoteInRobot", hasNote);
        Logger.recordOutput("Sim/Robot/NotesLaunched", notesLaunched);
        Logger.recordOutput("Sim/Robot/NotesScored", notesScored);
        Logger.recordOutput("Sim/Robot/IntakeRps", velocity(intakeMotor));
        var body = Robot.swerve.getSimDrivetrain().getBody();
        var speeds = body.getDriveTrainSimulatedChassisSpeedsRobotRelative();
        Logger.recordOutput(
                "Sim/Robot/SpeedMps",
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        Logger.recordOutput("Sim/Robot/LauncherRps", velocity(launcherMotor));
        Logger.recordOutput("Sim/Robot/FeederSincePickup", hasNote ? feeder - feederAtPickup : 0);
    }

    /**
     * Launches a note from {@code from} straight at its alliance's speaker: the auto aims for it,
     * so only where the robot shoots from matters. The speed puts the arc through the opening when
     * it's aimed {@link #AIM_ABOVE_SIGHT_DEG} above the line of sight.
     */
    public static void aimedShot(Pose2d from, boolean red, Runnable onScore) {
        Translation3d speaker = red ? RED_SPEAKER : BLUE_SPEAKER;
        Translation2d toSpeaker = speaker.toTranslation2d().minus(from.getTranslation());
        double d = toSpeaker.getNorm(), rise = speaker.getZ() - LAUNCH_HEIGHT;
        double angle = Math.atan2(rise, d) + Math.toRadians(AIM_ABOVE_SIGHT_DEG);
        double cos = Math.cos(angle);
        double speed = Math.sqrt(9.81 * d * d / (2 * cos * cos * (d * Math.tan(angle) - rise)));
        arena.addGamePieceProjectile(
                new NoteOnFly(
                                from.getTranslation(),
                                new Translation2d(),
                                new ChassisSpeeds(),
                                toSpeaker.getAngle(),
                                Meters.of(LAUNCH_HEIGHT),
                                MetersPerSecond.of(speed),
                                Radians.of(angle))
                        .enableBecomeNoteOnFieldAfterTouchGround()
                        .withTargetPosition(() -> speaker)
                        .withTargetTolerance(SPEAKER_TOLERANCE)
                        .withHitTargetCallBack(onScore));
    }

    /** Where a spit lands, ahead of the robot's centre, m: just past the front bumper. */
    private static final double SPIT_DROP_M = 0.7;

    /** Spits a note out the front: it drops on the floor right in front of the robot. */
    public static void spit(Pose2d from) {
        arena.addGamePiece(
                new CrescendoNoteOnField(
                        from.transformBy(new Transform2d(SPIT_DROP_M, 0, Rotation2d.kZero))
                                .getTranslation()));
    }

    private static double velocity(MotorSim m) {
        return m == null ? 0 : m.talon.getVelocity().refresh().getValueAsDouble();
    }

    /** Mechanism rotations from the sim rotor, which re-zeroing the encoder doesn't move. */
    private static double position(MotorSim m) {
        return m == null ? 0 : m.rotor.getAngularPositionRotations() / m.sensorToMechanism;
    }

    private static MotorSim motorSim(TalonFX talon) {
        if (talon == null) {
            return null;
        }
        MotorSim m = new MotorSim(talon);
        motors.add(m);
        return m;
    }

    /**
     * A free-spinning Kraken (Phoenix's sim motor) behind a mechanism: enough for its commands to
     * finish.
     */
    private static class MotorSim {
        final TalonFX talon;
        final TalonFXSimState state;
        final double sensorToMechanism;
        final DCMotorSim rotor =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.0005, 1),
                        DCMotor.getKrakenX60(1));

        MotorSim(TalonFX talon) {
            this.talon = talon;
            state = talon.getSimState();
            MotorOutputConfigs output = new MotorOutputConfigs();
            talon.getConfigurator().refresh(output);
            FeedbackConfigs feedback = new FeedbackConfigs();
            talon.getConfigurator().refresh(feedback);
            sensorToMechanism = feedback.SensorToMechanismRatio;
            state.Orientation =
                    output.Inverted == InvertedValue.Clockwise_Positive
                            ? ChassisReference.Clockwise_Positive
                            : ChassisReference.CounterClockwise_Positive;
        }

        void update() {
            state.setSupplyVoltage(12);
            rotor.setInputVoltage(state.getMotorVoltage());
            rotor.update(0.02);
            state.setRawRotorPosition(rotor.getAngularPositionRotations());
            state.setRotorVelocity(rotor.getAngularVelocityRPM() / 60);
        }
    }
}
