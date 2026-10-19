package frc.robot.agent;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.Decisions;
import com.pathplanner.lib.auto.PathEdits;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The decider behind Agentic Rush's decision nodes. Each option is a centre note (C1 to C5), and a
 * loose note we can reach (one another robot spat out, say) counts as one more, reached by the
 * option whose centre note is nearest it, with that path's intake waypoint moved onto it and the
 * robot turned so its back intake leads into it. It takes the target it's surest it gets:
 *
 * <ol>
 *   <li>a note seen there that we reach before any robot heading for it,
 *   <li>then a note not seen yet that we'd still reach first,
 *   <li>then the same two again, with another robot getting there first,
 * </ol>
 *
 * <p>ties going to the nearest note. An opponent only counts as getting there first if it's driving
 * straight at the note or sitting on it. A note a partner gets to first the same way is off limits,
 * like one seen gone, and so is one designated off limits on the dashboard (Agent/Off
 * Limits/C1..C5, or UV_AGENT_OFF_LIMITS="C1,C2" headless) for a partner's auto to take. Any robot
 * near the option's scripted path (within {@link #PATH_CLEAR_M}) blocks it the same way. Notes seen
 * gone are never picked unless they all are. It always answers, so the robot never waits at a
 * decision.
 */
public final class RushAgent {
    /**
     * The first node (C3, C2, C4) and the later ones (all five). They're named apart because an
     * early decide: lock indexes into one node's option list.
     */
    public static final String FIRST = "First Decision", DECISION = "Decision";

    /** Speed used to compare who gets to a note first, m/s. Every robot is an Ultraviolet. */
    private static final double SPEED_MPS = 4.0;

    /**
     * How much sooner another robot must arrive to count as getting there first, s. None: arriving
     * together means a contested note, which we lose as often as not.
     */
    private static final double BEATEN_BY_S = 0;

    /**
     * A robot this close to an option's route blocks it, m: two half-bumpers (0.46 each) and some
     * room to pass.
     */
    private static final double PATH_CLEAR_M = 1.2;

    /** Each centre note's option path, blue-side, loaded when the agent registers. */
    private static final List<PathPlannerPath> routes = new ArrayList<>();

    /** Score added to a steal, so it beats a centre note that's just as good, a half tier. */
    private static final double STEAL_BONUS = 50;

    /** A robot this close to a note is taking it, m. */
    private static final double ON_IT_M = 0.6;

    /**
     * A robot counts as heading for a note above this speed, m/s, when the note is within this
     * angle of its direction of travel, deg. An angle rather than a straight line, since robots
     * curve in.
     */
    private static final double MOVING_MPS = 1.0, AIMED_AT_DEG = 35;

    /**
     * The loose note the next path to centre note {@link #stealNote} goes to instead, field
     * coordinates; null when the last decision wasn't a steal.
     */
    private static Translation2d stealTarget;

    private static int stealNote = -1;

    private RushAgent() {}

    /** Dashboard key that designates centre note {@code i} off limits, left to a partner. */
    private static String offLimitsKey(int i) {
        return "Agent/Off Limits/C" + (i + 1);
    }

    /** Whether centre note {@code i} is designated off limits, on the dashboard or headless. */
    private static boolean designatedOffLimits(int i) {
        String env = System.getenv("UV_AGENT_OFF_LIMITS");
        return SmartDashboard.getBoolean(offLimitsKey(i), false)
                || env != null && List.of(env.split(",")).contains("C" + (i + 1));
    }

    /**
     * Registers the deciders and the C3 path edit. Call before autos are built, like named
     * commands. {@code UV_AGENT=0} leaves them out, so each decision runs its default option: the
     * scripted baseline.
     */
    public static void register() {
        if ("0".equals(System.getenv("UV_AGENT"))) {
            return; // benchmark baseline: every decision takes its default, the scripted route
        }
        for (int i = 0; i < CenterLineMap.NOTE_Y.length; i++) {
            SmartDashboard.setDefaultBoolean(offLimitsKey(i), false);
        }
        Decisions.register(FIRST, ctx -> decide(ctx.optionNames()));
        Decisions.register(DECISION, ctx -> decide(ctx.optionNames()));
        routes.clear();
        for (int i = 0; i < CenterLineMap.NOTE_Y.length; i++) {
            int note = i;
            String name = "C" + (i + 1) + " Decision";
            PathEdits.register(name, path -> toStealTarget(path, note));
            PathPlannerPath loaded = null;
            try {
                loaded = PathPlannerPath.fromPathFile(name);
            } catch (Exception e) {
                DriverStation.reportWarning("RushAgent can't load " + name + ": " + e, false);
            }
            routes.add(loaded);
        }
        PathPlannerLogging.setLogDecisionCallback(
                o ->
                        Logger.recordOutput(
                                "Agent/Output/Outcome",
                                o.decisionName()
                                        + ": "
                                        + o.optionName()
                                        + (o.timedOut() ? " (timed out)" : "")));
    }

    static int decide(List<String> options) {
        Translation2d us = Robot.swerve.getPose().getTranslation();
        int best = -1;
        double bestScore = Double.NEGATIVE_INFINITY;
        Translation2d bestLoose = null;
        List<String> why = new ArrayList<>();
        for (int o = 0; o < options.size(); o++) {
            int i = noteIndex(options.get(o));
            if (i < 0) {
                why.add(options.get(o) + ": not a centre note");
                continue;
            }
            double score =
                    score(
                            options.get(o),
                            CenterLineMap.spot(i),
                            i,
                            false,
                            CenterLineMap.state(i),
                            us,
                            why);
            if (score > bestScore) {
                bestScore = score;
                best = o;
                bestLoose = null;
            }
        }
        // A loose note rides on the option whose centre note is nearest it, with that path's
        // intake waypoint moved onto it.
        for (Translation2d n : CenterLineMap.looseNotes()) {
            int via = -1;
            for (int o = 0; o < options.size(); o++) {
                int i = noteIndex(options.get(o));
                if (i >= 0
                        && (via < 0
                                || CenterLineMap.spot(i).getDistance(n)
                                        < CenterLineMap.spot(noteIndex(options.get(via)))
                                                .getDistance(n))) {
                    via = o;
                }
            }
            if (via < 0) {
                continue;
            }
            double score =
                    score(
                            "loose note via " + options.get(via),
                            n,
                            noteIndex(options.get(via)),
                            true,
                            CenterLineMap.State.PRESENT,
                            us,
                            why);
            if (score > bestScore) {
                bestScore = score;
                best = via;
                bestLoose = n;
            }
        }
        stealTarget = bestLoose;
        stealNote = bestLoose != null ? noteIndex(options.get(best)) : -1;

        Logger.recordOutput("Agent/Output/Considered", why.toArray(new String[0]));
        Logger.recordOutput(
                "Agent/Output/Choice",
                best < 0 ? "none" : options.get(best) + (bestLoose != null ? " (steal)" : ""));
        Translation2d target =
                bestLoose != null
                        ? bestLoose
                        : best >= 0 && noteIndex(options.get(best)) >= 0
                                ? CenterLineMap.spot(noteIndex(options.get(best)))
                                : null;
        if (target != null) {
            Logger.recordOutput("Agent/Output/ChoicePose", new Pose2d(target, Rotation2d.kZero));
        }
        return best < 0 ? 0 : best;
    }

    /**
     * Tiers 4 to 0 times 100, less our travel time so the nearest wins within a tier. A steal gets
     * half a tier more, so it wins its tier but never a better one.
     */
    private static double score(
            String name,
            Translation2d note,
            int path,
            boolean steal,
            CenterLineMap.State state,
            Translation2d us,
            List<String> why) {
        double ours = us.getDistance(note) / SPEED_MPS;
        double theirs = Double.POSITIVE_INFINITY, mates = Double.POSITIVE_INFINITY;
        double nearPath = Double.POSITIVE_INFINITY;
        for (CenterLineMap.Track r : CenterLineMap.robots()) {
            if (r.red == AutoBuilder.shouldFlip()) {
                mates = Math.min(mates, arrival(r, note));
            } else {
                theirs = Math.min(theirs, arrival(r, note));
            }
            nearPath = Math.min(nearPath, distanceToRoute(r.position, path, note, steal));
        }
        boolean beaten = theirs + BEATEN_BY_S < ours;
        boolean blocked = nearPath < PATH_CLEAR_M;
        boolean designated = !steal && designatedOffLimits(path);
        boolean offLimits = designated || mates <= ours;
        int tier =
                state == CenterLineMap.State.GONE || offLimits
                        ? 0
                        : (beaten || blocked ? 0 : 2)
                                + (state == CenterLineMap.State.PRESENT ? 2 : 1);
        double score = tier * 100 + (steal && tier > 0 ? STEAL_BONUS : 0) - ours;
        why.add(
                String.format(
                        "%s at (%.1f, %.1f): %s%s, us %.1f s, robots %s, path %s, score %.1f",
                        name,
                        note.getX(),
                        note.getY(),
                        state,
                        designated
                                ? ", off limits (designated)"
                                : offLimits
                                        ? String.format(
                                                ", off limits (teammate first in %.1f s)", mates)
                                        : "",
                        ours,
                        Double.isInfinite(theirs)
                                ? "none heading for it"
                                : String.format("first in %.1f s", theirs),
                        blocked
                                ? String.format("blocked (robot %.1f m from it)", nearPath)
                                : "clear",
                        score));
        return score;
    }

    /**
     * How close {@code robot} is to the route we'd drive for centre note {@code path}'s option: its
     * scripted path, or for a steal that path with its intake waypoint moved onto {@code note}.
     * Paths are drawn blue-side, so they're flipped for red.
     */
    private static double distanceToRoute(
            Translation2d robot, int path, Translation2d note, boolean steal) {
        PathPlannerPath route = routes.get(path);
        if (route == null) {
            return Double.POSITIVE_INFINITY;
        }
        boolean red = AutoBuilder.shouldFlip();
        if (steal) {
            route = movedTo(route, path, red ? FlippingUtil.flipFieldPosition(note) : note);
        }
        double closest = robot.getDistance(note);
        for (Pose2d p : route.getPathPoses()) {
            Translation2d t = p.getTranslation();
            closest =
                    Math.min(
                            closest,
                            robot.getDistance(red ? FlippingUtil.flipFieldPosition(t) : t));
        }
        return closest;
    }

    /**
     * When robot {@code r} gets to {@code note}: now if it's on it, after the drive if it's driving
     * straight at it, never if it's going somewhere else.
     */
    private static double arrival(CenterLineMap.Track r, Translation2d note) {
        Translation2d toNote = note.minus(r.position);
        if (toNote.getNorm() < ON_IT_M) {
            return 0;
        }
        double speed = r.velocity.getNorm();
        if (speed < MOVING_MPS) {
            return Double.POSITIVE_INFINITY;
        }
        double cos =
                (toNote.getX() * r.velocity.getX() + toNote.getY() * r.velocity.getY())
                        / (toNote.getNorm() * speed);
        return cos > Math.cos(Math.toRadians(AIMED_AT_DEG))
                ? toNote.getNorm() / SPEED_MPS
                : Double.POSITIVE_INFINITY;
    }

    /**
     * The path to centre note {@code note}, with its intake waypoint moved onto the loose note when
     * the last decision chose to steal it via this path; unchanged otherwise. Paths are drawn
     * blue-side, so a red target is flipped first.
     */
    private static PathPlannerPath toStealTarget(PathPlannerPath path, int note) {
        if (stealTarget == null || stealNote != note) {
            return path;
        }
        Translation2d target = stealTarget;
        stealTarget = null;
        stealNote = -1;
        if (AutoBuilder.shouldFlip()) {
            target = FlippingUtil.flipFieldPosition(target);
        }
        PathPlannerPath moved = movedTo(path, note, target);
        Pose2d intake = new Pose2d(target, Rotation2d.kZero);
        Logger.recordOutput("Agent/Output/StealWaypoint", intake);
        return moved;
    }

    /**
     * {@code path} (to centre note {@code note}) with its intake waypoint moved onto {@code
     * target}, blue-side, and the back intake pointed at it on the way in.
     */
    private static PathPlannerPath movedTo(PathPlannerPath path, int note, Translation2d target) {
        List<Waypoint> waypoints = new ArrayList<>(path.getWaypoints());
        Translation2d spot = CenterLineMap.spot(note);
        int intake = 0;
        for (int i = 1; i < waypoints.size(); i++) {
            if (waypoints.get(i).anchor().getDistance(spot)
                    < waypoints.get(intake).anchor().getDistance(spot)) {
                intake = i;
            }
        }
        Waypoint w = waypoints.get(intake);
        Translation2d shift = target.minus(w.anchor());
        waypoints.set(
                intake,
                new Waypoint(
                        w.prevControl() == null ? null : w.prevControl().plus(shift),
                        target,
                        w.nextControl() == null ? null : w.nextControl().plus(shift)));
        // Keep the back intake pointed at the note all the way in. A zone overrides rotation
        // targets in it; drop the ones near the intake too, which were set for the old spot.
        List<RotationTarget> rotations = new ArrayList<>();
        for (RotationTarget r : path.getRotationTargets()) {
            if (Math.abs(r.position() - intake) > 0.3) {
                rotations.add(r);
            }
        }
        List<PointTowardsZone> zones = new ArrayList<>(path.getPointTowardsZones());
        // Stops just short of the note, where the angle to it is undefined.
        zones.add(new PointTowardsZone("Steal", target, Rotation2d.k180deg, 0, intake - 0.05));
        return new PathPlannerPath(
                waypoints,
                rotations,
                zones,
                path.getConstraintZones(),
                path.getEventMarkers(),
                path.getGlobalConstraints(),
                path.getIdealStartingState(),
                path.getGoalEndState(),
                path.isReversed());
    }

    /** "C3" is note 2 (0-based); anything else isn't a centre note. */
    private static int noteIndex(String option) {
        if (option.length() == 2 && option.charAt(0) == 'C') {
            int i = option.charAt(1) - '1';
            return i >= 0 && i < CenterLineMap.NOTE_Y.length ? i : -1;
        }
        return -1;
    }
}
