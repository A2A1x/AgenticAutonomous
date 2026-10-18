package com.pathplanner.lib.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Registry of the deciders used by decision nodes in PathPlanner autos */
public class Decisions {
  /** Picks one option of a decision node */
  @FunctionalInterface
  public interface Decider {
    /**
     * Pick an option
     *
     * @param context Information about the decision being made
     * @return The index of the chosen option, or -1 if not decided yet
     */
    int decide(Context context);
  }

  /**
   * Information given to a decider
   *
   * @param optionNames The names of the decision's options, in order
   * @param secondsSinceReached Seconds since the decision node was reached. 0 when asked early by a
   *     decide:&lt;name&gt; event marker
   */
  public record Context(List<String> optionNames, double secondsSinceReached) {}

  /**
   * The result of a decision
   *
   * @param decisionName The name of the decision node
   * @param optionIndex The index of the chosen option
   * @param optionName The name of the chosen option
   * @param timedOut True if no choice was made before the timeout and the default option was used
   */
  public record Outcome(
      String decisionName, int optionIndex, String optionName, boolean timedOut) {}

  private static final Map<String, Decider> deciders = new HashMap<>();
  private static final Map<String, List<String>> optionNames = new HashMap<>();
  private static final Map<String, Integer> locks = new HashMap<>();

  /**
   * Register the decider for decision nodes with the given name. This also registers the named
   * command "decide:&lt;name&gt;", which asks the decider early and locks in its choice. Place it
   * as an event marker on a path before the decision node so the robot decides without stopping.
   * Like other named commands, register deciders before building autos.
   *
   * @param name The decision name
   * @param decider The decider
   */
  public static void register(String name, Decider decider) {
    deciders.put(name, decider);
    NamedCommands.registerCommand("decide:" + name, Commands.runOnce(() -> lock(name)));
  }

  /** Remove all deciders, known decision nodes and locked choices */
  public static void clearAll() {
    deciders.clear();
    optionNames.clear();
    locks.clear();
  }

  /**
   * Record the option names of a decision node so decide:&lt;name&gt; markers can build a context.
   * This is used internally. If several autos use the same decision name, the last one built wins.
   *
   * @param name The decision name
   * @param options The option names
   */
  public static void registerOptions(String name, List<String> options) {
    optionNames.put(name, options);
  }

  /**
   * Ask the decider registered under the given name. This is used internally.
   *
   * @param name The decision name
   * @param context The context to pass to the decider
   * @return The chosen option index, or -1 if undecided, no decider is registered, the index is out
   *     of range, or the decider threw
   */
  public static int ask(String name, Context context) {
    Decider decider = deciders.get(name);
    if (decider == null) {
      DriverStation.reportWarning(
          "PathPlanner decision '" + name + "' has no decider registered with Decisions.register",
          false);
      return -1;
    }
    try {
      int choice = decider.decide(context);
      return (choice >= 0 && choice < context.optionNames().size()) ? choice : -1;
    } catch (RuntimeException e) {
      DriverStation.reportError(
          "PathPlanner decider '" + name + "' threw an exception", e.getStackTrace());
      return -1;
    }
  }

  /**
   * Check if a decider is registered under the given name
   *
   * @param name The decision name
   * @return True if a decider is registered
   */
  public static boolean hasDecider(String name) {
    return deciders.containsKey(name);
  }

  /**
   * Take the choice locked in by a decide:&lt;name&gt; marker, clearing it. This is used
   * internally.
   *
   * @param name The decision name
   * @return The locked option index, or null if nothing is locked
   */
  public static Integer takeLock(String name) {
    return locks.remove(name);
  }

  /** Clear all locked choices. This is used internally when an auto starts. */
  public static void clearLocks() {
    locks.clear();
  }

  private static void lock(String name) {
    List<String> names = optionNames.get(name);
    if (names == null) {
      DriverStation.reportWarning(
          "PathPlanner marker 'decide:" + name + "' ran, but no decision node has that name",
          false);
      return;
    }
    int choice = ask(name, new Context(names, 0));
    if (choice >= 0) {
      locks.put(name, choice);
    }
  }
}
