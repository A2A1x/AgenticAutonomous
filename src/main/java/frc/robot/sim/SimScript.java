package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * Runs autos headless, for benchmarks. {@code UV_SIM_AUTO} names the PathPlanner auto, {@code
 * UV_SIM_AUTO_RUNS} how many times to run it (1 s disabled, then 15 s of auto each), and {@code
 * UV_SIM_EXIT=1} quits the sim after the last run.
 */
public final class SimScript {
    private static final String AUTO = System.getenv("UV_SIM_AUTO");
    private static final int RUNS = parseOr(System.getenv("UV_SIM_AUTO_RUNS"), 1);
    private static final boolean EXIT = "1".equals(System.getenv("UV_SIM_EXIT"));
    /** Seconds disabled before each run; UV_SIM_DISABLED_SECONDS lengthens it. */
    private static final double DISABLED_S = parseOr(System.getenv("UV_SIM_DISABLED_SECONDS"), 1.0);
    /** Seconds of auto per run; UV_SIM_AUTO_SECONDS shortens it, like disabling mid-auto. */
    private static final double AUTO_S = parseOr(System.getenv("UV_SIM_AUTO_SECONDS"), 15);

    private static double start = -1;

    private SimScript() {}

    /** The auto to run, or null when no script is set. */
    public static String auto() {
        return AUTO;
    }

    public static void periodic() {
        if (AUTO == null) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        if (start < 0) {
            start = now;
        }
        double t = now - start;
        int run = (int) (t / (DISABLED_S + AUTO_S));
        boolean enabled = run < RUNS && t - run * (DISABLED_S + AUTO_S) >= DISABLED_S;
        if (run >= RUNS && EXIT) {
            System.exit(0);
        }
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    private static int parseOr(String s, int fallback) {
        return (int) parseOr(s, (double) fallback);
    }

    private static double parseOr(String s, double fallback) {
        try {
            return s == null ? fallback : Double.parseDouble(s.trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }
}
