package frc.spectrumLib.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.spectrumLib.swerve.config.ModuleConfig;
import frc.spectrumLib.swerve.config.SwerveConfig;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * The drivetrain as a maple-sim body: wheels that slip, a bumper that collides with notes and
 * robots, and a true pose apart from odometry.
 *
 * <p>maple-sim's motor models drive the real TalonFX and CANcoder sim states, so the module and
 * odometry code runs unchanged. The arena steps the body; {@link #update()} copies its gyro into
 * the Pigeon 2.
 */
public class MapleSimDrivetrain {
    private final SwerveDriveSimulation body;
    private final GyroSimulation gyro;
    private final Pigeon2SimState pigeon;

    public MapleSimDrivetrain(SwerveConfig config, Module[] modules, Pigeon2 pigeon2) {
        body = new SwerveDriveSimulation(driveTrainConfig(config), new Pose2d());
        gyro = body.getGyroSimulation();
        pigeon = pigeon2.getSimState();

        for (int i = 0; i < modules.length; i++) {
            ModuleConfig c = config.modules[i];
            body.getModules()[i].useDriveMotorController(
                    new TalonFXSim(modules[i].getDriveMotor(), c.DriveMotorInverted));
            body.getModules()[i].useSteerMotorController(
                    new SteerSim(
                            modules[i].getSteerMotor(),
                            c.SteerMotorInverted,
                            modules[i].getCANcoder(),
                            c.CANcoderOffset));
        }
        SimulatedArena.getInstance().addDriveTrainSimulation(body);
    }

    /** maple-sim's model of a drivetrain built to {@code config}; sim robots reuse it. */
    public static DriveTrainSimulationConfig driveTrainConfig(SwerveConfig config) {
        ModuleConfig module = config.modules[0];
        Translation2d[] locations = new Translation2d[config.modules.length];
        for (int i = 0; i < locations.length; i++) {
            locations[i] =
                    new Translation2d(config.modules[i].LocationX, config.modules[i].LocationY);
        }
        SwerveModuleSimulationConfig moduleSim =
                new SwerveModuleSimulationConfig(
                        // Phoenix's sim models every TalonFX as a Kraken X60, so torque-current
                        // control only lands where it aims if maple uses the same motor.
                        DCMotor.getKrakenX60Foc(1),
                        DCMotor.getKrakenX60Foc(1),
                        module.DriveMotorGearRatio,
                        module.SteerMotorGearRatio,
                        Volts.of(0.1),
                        Volts.of(0.2),
                        Meters.of(edu.wpi.first.math.util.Units.inchesToMeters(module.WheelRadius)),
                        KilogramSquareMeters.of(0.025),
                        config.wheelCOF);
        return DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(config.robotMassKg))
                .withBumperSize(
                        Meters.of(config.bumperLengthMeters), Meters.of(config.bumperWidthMeters))
                .withCustomModuleTranslations(locations)
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(moduleSim);
    }

    /** Copies the simulated gyro into the Pigeon 2. Call once a loop. */
    public void update() {
        pigeon.setSupplyVoltage(12);
        pigeon.setRawYaw(gyro.getGyroReading().getMeasure());
        pigeon.setAngularVelocityZ(gyro.getMeasuredAngularVelocity());
    }

    /** Where the robot really is, unlike odometry. */
    public Pose2d getTruePose() {
        return body.getSimulatedDriveTrainPose();
    }

    /** Puts the robot at {@code pose}, as a pose reset does on the field. */
    public void setPose(Pose2d pose) {
        body.setSimulationWorldPose(pose);
    }

    public SwerveDriveSimulation getBody() {
        return body;
    }

    /** A TalonFX whose rotor follows maple-sim's motor model. */
    private static class TalonFXSim implements SimulatedMotorController {
        private final TalonFXSimState sim;

        TalonFXSim(TalonFX talon, boolean inverted) {
            sim = talon.getSimState();
            sim.Orientation =
                    inverted
                            ? ChassisReference.Clockwise_Positive
                            : ChassisReference.CounterClockwise_Positive;
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            sim.setSupplyVoltage(12);
            sim.setRawRotorPosition(encoderAngle);
            sim.setRotorVelocity(encoderVelocity);
            return sim.getMotorVoltageMeasure();
        }
    }

    /** A steer TalonFX plus the CANcoder on its module, which sees the module angle. */
    private static class SteerSim extends TalonFXSim {
        private final CANcoderSimState cancoder;
        private final double offsetRotations;

        SteerSim(TalonFX talon, boolean inverted, CANcoder cancoder, double offsetRotations) {
            super(talon, inverted);
            this.cancoder = cancoder.getSimState();
            this.offsetRotations = offsetRotations;
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            cancoder.setSupplyVoltage(12);
            // The CANcoder adds its magnet offset, so the raw reading must leave it out.
            cancoder.setRawPosition(mechanismAngle.in(Rotations) - offsetRotations);
            cancoder.setVelocity(mechanismVelocity);
            return super.updateControlSignal(
                    mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }
}
