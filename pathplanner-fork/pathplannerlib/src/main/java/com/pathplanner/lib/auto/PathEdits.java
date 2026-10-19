package com.pathplanner.lib.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import java.util.HashMap;
import java.util.Map;
import java.util.function.UnaryOperator;

/**
 * Edits to paths in autos, made when each path starts rather than when the auto is built. An auto
 * follows a path with an edit registered through a deferred command, so the edit can use what the
 * robot knows at that moment, such as moving an intake waypoint to a note it just saw. Like named
 * commands, register edits before building autos.
 */
public class PathEdits {
  private static final Map<String, UnaryOperator<PathPlannerPath>> edits = new HashMap<>();

  /**
   * Register an edit for every use of a path in autos built after this call
   *
   * @param pathName The path's name in the GUI
   * @param edit Given the path as loaded (already mirrored if the auto is), returns the path to
   *     follow; return it unchanged to follow it as drawn
   */
  public static void register(String pathName, UnaryOperator<PathPlannerPath> edit) {
    edits.put(pathName, edit);
  }

  /**
   * Get the edit registered for a path. This is used internally.
   *
   * @param pathName The path's name
   * @return The edit, or null if none is registered
   */
  public static UnaryOperator<PathPlannerPath> get(String pathName) {
    return edits.get(pathName);
  }

  /** Remove all registered edits */
  public static void clearAll() {
    edits.clear();
  }
}
