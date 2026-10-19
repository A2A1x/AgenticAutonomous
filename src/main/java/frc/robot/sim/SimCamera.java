package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.agent.Detections;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.ironmaple.simulation.SimulatedArena;

/**
 * Ultraviolet's note detection in sim: it sees notes and robots all the way round (field of view
 * and note range from {@link Detections}) and misses some frames. What it sees comes from the true
 * pose; where it puts it on the field uses odometry, as the real pipeline would.
 */
public final class SimCamera {
    /**
     * The view's centre, out the back (the intake side); with a 360-degree view it only sets the
     * logged heading.
     */
    private static final Rotation2d FACING = Rotation2d.k180deg;

    private static final double ROBOT_RANGE_M = 10.0;

    /** Chance a note or robot in view is detected in a frame, and position noise, m. */
    private static final double DETECT_P = 0.9, NOISE_M = 0.05;

    private static final Random noise = new Random(3847);

    private SimCamera() {}

    public static Detections frame() {
        Pose2d truth = Robot.swerve.getSimDrivetrain().getTruePose();
        Pose2d odometry = Robot.swerve.getPose();
        Pose2d camera = new Pose2d(truth.getTranslation(), truth.getRotation().plus(FACING));
        List<Translation2d> notes = new ArrayList<>();
        for (Pose3d n : SimulatedArena.getInstance().getGamePiecesArrayByType("Note")) {
            if (n.getZ() < 0.2
                    && inView(camera, n.toPose2d().getTranslation(), Detections.NOTE_RANGE_M)) {
                notes.add(placed(truth, odometry, n.toPose2d().getTranslation()));
            }
        }
        List<Translation2d> robots = new ArrayList<>();
        List<Boolean> robotsRed = new ArrayList<>();
        List<Pose2d> others = SimRobots.onField();
        List<Boolean> othersRed = SimRobots.onFieldRed();
        for (int i = 0; i < others.size(); i++) {
            Translation2d r = others.get(i).getTranslation();
            if (inView(camera, r, ROBOT_RANGE_M)) {
                robots.add(placed(truth, odometry, r));
                robotsRed.add(othersRed.get(i)); // bumper colour
            }
        }
        return new Detections(camera(odometry), notes, robots, robotsRed);
    }

    /** Where the camera thinks it is: odometry, facing out the back. */
    private static Pose2d camera(Pose2d odometry) {
        return new Pose2d(odometry.getTranslation(), odometry.getRotation().plus(FACING));
    }

    private static boolean inView(Pose2d camera, Translation2d target, double range) {
        Translation2d rel = target.minus(camera.getTranslation());
        double off = rel.getAngle().minus(camera.getRotation()).getRadians();
        return rel.getNorm() < range
                && Math.abs(off) <= Detections.HALF_FOV_RAD
                && noise.nextDouble() < DETECT_P;
    }

    /** {@code target} seen from the true pose, then put on the field from odometry. */
    private static Translation2d placed(Pose2d truth, Pose2d odometry, Translation2d target) {
        Transform2d seen = new Pose2d(target, Rotation2d.kZero).minus(truth);
        return odometry.transformBy(seen)
                .getTranslation()
                .plus(
                        new Translation2d(
                                noise.nextGaussian() * NOISE_M, noise.nextGaussian() * NOISE_M));
    }
}
