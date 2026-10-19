package frc.robot.agent;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/** One camera frame: where the camera was, and the notes and robots it saw, on the field. */
public class Detections {
    /** The note cameras' view: all the way round (360 degrees), out to the note range. */
    public static final double HALF_FOV_RAD = Math.PI, NOTE_RANGE_M = 5.0;

    public static final Detections NONE = new Detections(null, List.of(), List.of(), List.of());

    /** The camera's field pose, facing where it looks; null when there's no camera. */
    public final Pose2d camera;

    public final List<Translation2d> notes;
    public final List<Translation2d> robots;

    /** Each robot's bumper colour, in the same order as {@link #robots}: true for red. */
    public final List<Boolean> robotsRed;

    public Detections(
            Pose2d camera,
            List<Translation2d> notes,
            List<Translation2d> robots,
            List<Boolean> robotsRed) {
        this.camera = camera;
        this.notes = notes;
        this.robots = robots;
        this.robotsRed = robotsRed;
    }

    /**
     * The camera's view as a closed outline, for the field view: a circle when it sees all round.
     */
    public Pose2d[] viewOutline() {
        int steps = 24;
        boolean allRound = HALF_FOV_RAD >= Math.PI;
        List<Pose2d> outline = new ArrayList<>();
        if (!allRound) {
            outline.add(camera);
        }
        for (int i = 0; i <= steps; i++) {
            Rotation2d dir =
                    camera.getRotation()
                            .plus(new Rotation2d(-HALF_FOV_RAD + i * 2 * HALF_FOV_RAD / steps));
            outline.add(
                    new Pose2d(
                            camera.getTranslation().plus(new Translation2d(NOTE_RANGE_M, dir)),
                            dir));
        }
        if (!allRound) {
            outline.add(camera);
        }
        return outline.toArray(Pose2d[]::new);
    }

    /** Whether {@code spot} is close enough and inside the view to see a note there. */
    public boolean covers(Translation2d spot) {
        if (camera == null) {
            return false;
        }
        Translation2d rel = spot.minus(camera.getTranslation());
        return rel.getNorm() < NOTE_RANGE_M
                && Math.abs(rel.getAngle().minus(camera.getRotation()).getRadians())
                        <= HALF_FOV_RAD;
    }
}
