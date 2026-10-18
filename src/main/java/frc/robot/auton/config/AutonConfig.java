package frc.robot.auton.config;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.auton.Auton;
import frc.robot.swerve.commands.ApplyChassisSpeeds;
import frc.spectrumLib.swerve.config.ModuleConfig;
import java.util.Optional;

public class AutonConfig {

    public static final double kTranslationP = 5;
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;
    public static final double kRotationP = 5;
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.0;
    public static final double maxModuleSpeed = 6;

    /** Estimated: about 125 lb of robot plus bumpers and battery. Weigh the robot to refine it. */
    public static final double kRobotMassKg = 63.5;

    /** Estimated as a uniform 28 in square of {@link #kRobotMassKg}. */
    public static final double kRobotMOI = kRobotMassKg * 2 * Math.pow(0.711, 2) / 12;

    /** Rubber-tread default; the 2024 code never measured it. */
    public static final double kWheelCOF = 1.2;

    public static boolean commandInit = false;

    public static final PPHolonomicDriveController AutonPathFollowerController =
            new PPHolonomicDriveController(
                    new PIDConstants(kTranslationP, kTranslationI, kTranslationD),
                    new PIDConstants(kRotationP, kRotationI, kRotationD));

    /** PathPlanner's model of Ultraviolet, built from the swerve config it already has. */
    public static com.pathplanner.lib.config.RobotConfig robotConfig() {
        ModuleConfig[] modules = Robot.swerve.config.modules;
        ModuleConfig module = modules[0];
        Translation2d[] locations = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            locations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
        }
        return new com.pathplanner.lib.config.RobotConfig(
                kRobotMassKg,
                kRobotMOI,
                new com.pathplanner.lib.config.ModuleConfig(
                        Units.inchesToMeters(module.WheelRadius),
                        maxModuleSpeed,
                        kWheelCOF,
                        // Falcon 500s, matching SimDrivetrain
                        DCMotor.getFalcon500(1).withReduction(module.DriveMotorGearRatio),
                        module.SlipCurrent,
                        1),
                locations);
    }

    public static void configureAutoBuilder() {
        // Set the method that will be used to get rotation overrides
        PPHolonomicDriveController.setRotationTargetOverride(() -> getRotationTargetOverride());

        // Configure AutoBuilder last
        AutoBuilder.configure(
                Robot.swerve::getPose, // Robot pose supplier
                Robot.swerve::resetPose, // Method to reset odometry (will be called if your auto
                // has a starting pose)
                Robot.swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
                // RELATIVE
                ApplyChassisSpeeds.robotRelativeOutput(
                        false), // Method that will drive the robot given ROBOT
                // RELATIVE
                // ChassisSpeeds
                AutonPathFollowerController,
                robotConfig(),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                Robot.swerve // Reference to this subsystem to set requirements
                );
    }

    public static Optional<Rotation2d> getRotationTargetOverride() {
        // Some condition that should decide if we want to override rotation
        if (Auton.trackSpeaker) {
            // Return an optional containing the rotation override (this should be a field relative
            return Optional.of(Rotation2d.fromRadians(Robot.vision.getThetaToSpeaker())); //
        } else {
            return Optional
                    .empty(); // return an empty optional when we don't want to override the path's
            // rotation
        }
    }
}
