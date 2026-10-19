package frc.robot.agent;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * What the robot knows about the middle of the field: for each of C1 (amp side) to C5, whether a
 * note was last seen there or seen gone; loose notes we could still reach, like one another robot
 * spat out; and the robots seen, with how fast they're moving and where. Notes we pick up near a
 * spot mark it gone too, so it works without a camera.
 */
public final class CenterLineMap {
    public static final double CENTER_LINE_X = 8.27;
    public static final double[] NOTE_Y = {7.46, 5.78, 4.10, 2.43, 0.75};

    /** How near a detection must be to a spot to count as its note, m. */
    private static final double MATCH_M = 0.6;

    /** Frames in a row a covered spot must look empty before it's called gone. */
    private static final int EMPTY_FRAMES = 5;

    /**
     * Loose notes worth going for, in our alliance's x, m: past our wing and short of theirs. The
     * robot may cross the centre line to get one.
     */
    private static final double LOOSE_MIN_X = 5.5, LOOSE_MAX_X = 2 * CENTER_LINE_X - LOOSE_MIN_X;

    /** Loose notes and robots not seen for this long are forgotten, s. */
    private static final double LOOSE_MEMORY_S = 0.5, ROBOT_MEMORY_S = 1.0;

    /** Robot detections this close to a track from one frame to the next are the same robot, m. */
    private static final double TRACK_MATCH_M = 0.75;

    /**
     * Velocities are measured over this long, s: frame to frame, 5 cm of detection noise is metres
     * per second of velocity noise.
     */
    private static final double VELOCITY_WINDOW_S = 0.25;

    public enum State {
        UNKNOWN("Unknown"),
        PRESENT("Present"),
        GONE("Gone");

        /** Its key under Agent/Map/. */
        final String label;

        State(String label) {
            this.label = label;
        }
    }

    /** A robot followed from frame to frame. */
    public static final class Track {
        public Translation2d position;
        public Translation2d velocity = new Translation2d();
        /** Its bumper colour: true for red. */
        public boolean red;

        private double seenAt;
        /** Where it was seen over the last VELOCITY_WINDOW_S, oldest first, and when. */
        private final List<Translation2d> history = new ArrayList<>();

        private final List<Double> historyAt = new ArrayList<>();

        Track(Translation2d position, double seenAt) {
            this.position = position;
            this.seenAt = seenAt;
            history.add(position);
            historyAt.add(seenAt);
        }
    }

    private static final State[] state = new State[NOTE_Y.length];
    private static final int[] emptyFrames = new int[NOTE_Y.length];
    private static final List<Track> robots = new ArrayList<>();
    private static final List<Translation2d> loose = new ArrayList<>();
    private static final List<Double> looseTimes = new ArrayList<>();
    private static boolean hadNote;

    static {
        reset();
    }

    private CenterLineMap() {}

    public static Translation2d spot(int i) {
        return new Translation2d(CENTER_LINE_X, NOTE_Y[i]);
    }

    public static State state(int i) {
        return state[i];
    }

    public static List<Track> robots() {
        return robots;
    }

    /** Loose notes seen lately that we could reach, field coordinates. */
    public static List<Translation2d> looseNotes() {
        return loose;
    }

    /** Forgets everything, for the start of auto. */
    public static void reset() {
        for (int i = 0; i < state.length; i++) {
            state[i] = State.UNKNOWN;
            emptyFrames[i] = 0;
        }
        robots.clear();
        loose.clear();
        looseTimes.clear();
        hadNote = false;
    }

    /** Takes in a camera frame and our own pickups. Call once a loop on the main thread. */
    public static void update(Detections frame) {
        double now = Timer.getFPGATimestamp();
        for (int i = 0; i < state.length; i++) {
            Translation2d spot = spot(i);
            boolean seen = false;
            for (Translation2d n : frame.notes) {
                seen |= n.getDistance(spot) < MATCH_M;
            }
            if (seen) {
                state[i] = State.PRESENT;
                emptyFrames[i] = 0;
            } else if (frame.covers(spot) && ++emptyFrames[i] >= EMPTY_FRAMES) {
                state[i] = State.GONE;
            }
        }
        updateLoose(frame.notes, now);
        updateRobots(frame.robots, frame.robotsRed, now);

        // A note we just picked up near a spot was that spot's note.
        boolean hasNote = Robot.feeder.intakedNote();
        if (hasNote && !hadNote) {
            Pose2d us = Robot.swerve.getPose();
            for (int i = 0; i < state.length; i++) {
                if (us.getTranslation().getDistance(spot(i)) < 1.0) {
                    state[i] = State.GONE;
                }
            }
        }
        hadNote = hasNote;

        log(frame);
    }

    private static void updateLoose(List<Translation2d> notes, double now) {
        boolean red = AutoBuilder.shouldFlip();
        for (Translation2d n : notes) {
            double x = red ? 2 * CENTER_LINE_X - n.getX() : n.getX();
            boolean atSpot = false;
            for (int i = 0; i < state.length; i++) {
                atSpot |= n.getDistance(spot(i)) < MATCH_M;
            }
            if (atSpot || x < LOOSE_MIN_X || x > LOOSE_MAX_X) {
                continue;
            }
            int known = -1;
            for (int j = 0; j < loose.size(); j++) {
                if (loose.get(j).getDistance(n) < MATCH_M) {
                    known = j;
                }
            }
            if (known >= 0) {
                loose.set(known, n);
                looseTimes.set(known, now);
            } else {
                loose.add(n);
                looseTimes.add(now);
            }
        }
        for (int j = loose.size() - 1; j >= 0; j--) {
            if (now - looseTimes.get(j) > LOOSE_MEMORY_S) {
                loose.remove(j);
                looseTimes.remove(j);
            }
        }
    }

    /** Matches each robot seen to the nearest track, so each track knows its velocity. */
    private static void updateRobots(List<Translation2d> seen, List<Boolean> seenRed, double now) {
        List<Track> matched = new ArrayList<>();
        for (int s = 0; s < seen.size(); s++) {
            Translation2d r = seen.get(s);
            Track best = null;
            for (Track t : robots) {
                if (!matched.contains(t)
                        && t.position.getDistance(r) < TRACK_MATCH_M
                        && (best == null
                                || t.position.getDistance(r) < best.position.getDistance(r))) {
                    best = t;
                }
            }
            if (best == null) {
                best = new Track(r, now);
                robots.add(best);
            } else {
                best.history.add(r);
                best.historyAt.add(now);
                while (now - best.historyAt.get(1) >= VELOCITY_WINDOW_S) {
                    best.history.remove(0);
                    best.historyAt.remove(0);
                }
                double dt = now - best.historyAt.get(0);
                if (dt >= VELOCITY_WINDOW_S / 2) {
                    best.velocity = r.minus(best.history.get(0)).div(dt);
                }
                best.position = r;
                best.seenAt = now;
            }
            best.red = seenRed.get(s);
            matched.add(best);
        }
        robots.removeIf(t -> now - t.seenAt > ROBOT_MEMORY_S);
    }

    private static void log(Detections frame) {
        // What it takes in: this frame's camera view and detections
        Logger.recordOutput("Agent/Input/SeenNotes", poses(frame.notes));
        Logger.recordOutput("Agent/Input/SeenRobots", poses(frame.robots));
        if (frame.camera != null) {
            Logger.recordOutput("Agent/Input/CameraPose", frame.camera);
            Logger.recordOutput("Agent/Input/CameraView", frame.viewOutline());
        }
        // What it believes: each centre spot by state, loose notes, and robots with velocities
        String[] states = new String[state.length];
        for (int i = 0; i < state.length; i++) {
            states[i] = "C" + (i + 1) + " " + state[i];
        }
        Logger.recordOutput("Agent/Map/States", states);
        for (State s : State.values()) {
            List<Translation2d> spots = new ArrayList<>();
            for (int i = 0; i < state.length; i++) {
                if (state[i] == s) {
                    spots.add(spot(i));
                }
            }
            Logger.recordOutput("Agent/Map/" + s.label, poses(spots));
        }
        Logger.recordOutput("Agent/Map/LooseNotes", poses(loose));
        List<Translation2d> positions = new ArrayList<>();
        Pose2d[] headings = new Pose2d[robots.size()];
        for (int i = 0; i < robots.size(); i++) {
            Track t = robots.get(i);
            positions.add(t.position);
            headings[i] = new Pose2d(t.position, t.velocity.getAngle());
        }
        Logger.recordOutput("Agent/Map/Robots", poses(positions));
        // Each robot facing the way it's moving, for an arrow view of where it's heading.
        Logger.recordOutput("Agent/Map/RobotHeadings", headings);
    }

    private static Pose2d[] poses(List<Translation2d> points) {
        return points.stream()
                .map(t -> new Pose2d(t, Robot.swerve.getPose().getRotation()))
                .toArray(Pose2d[]::new);
    }
}
