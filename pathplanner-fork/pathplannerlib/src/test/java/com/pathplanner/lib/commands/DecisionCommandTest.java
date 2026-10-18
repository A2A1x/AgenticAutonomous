package com.pathplanner.lib.commands;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.Decisions;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DecisionCommandTest {
  private final List<Decisions.Outcome> outcomes = new ArrayList<>();

  @BeforeEach
  public void setup() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    Decisions.clearAll();
    NamedCommands.clearAll();
    outcomes.clear();
    PathPlannerLogging.setLogDecisionCallback(outcomes::add);
  }

  @AfterEach
  public void teardown() {
    PathPlannerLogging.clearLoggingCallbacks();
    SimHooks.resumeTiming();
  }

  private static JSONObject decisionJson(double timeout, int defaultOption) throws Exception {
    return (JSONObject)
        new JSONParser()
            .parse(
                """
                {"type": "decision", "data": {"name": "note", "timeout": %s, "defaultOption": %d,
                  "options": [
                    {"name": "left", "commands": [{"type": "wait", "data": {"waitTime": 0.0}}]},
                    {"name": "right", "commands": []}
                  ]}}
                """
                    .formatted(timeout, defaultOption));
  }

  private static Command parse(JSONObject json) throws Exception {
    return CommandUtil.commandFromJson(json, false, false);
  }

  @Test
  public void testParsesDecisionNode() throws Exception {
    Command cmd = parse(decisionJson(1.0, 1));
    assertInstanceOf(DecisionCommand.class, cmd);

    // No decider registered: the parsed default option runs right away
    cmd.initialize();
    assertEquals(List.of(new Decisions.Outcome("note", 1, "right", true)), outcomes);
    cmd.execute();
    assertTrue(cmd.isFinished());
  }

  @Test
  public void testEmptyOptionsParseToNone() throws Exception {
    JSONObject json = decisionJson(1.0, 0);
    ((List<?>) ((JSONObject) json.get("data")).get("options")).clear();
    assertFalse(parse(json) instanceof DecisionCommand);
  }

  @Test
  public void testDeciderPicks() throws Exception {
    Decisions.register("note", ctx -> ctx.secondsSinceReached() >= 0.1 ? 1 : -1);
    Command cmd = parse(decisionJson(1.0, 0));

    cmd.initialize();
    assertTrue(outcomes.isEmpty());
    cmd.execute();
    assertFalse(cmd.isFinished());

    SimHooks.stepTiming(0.2);
    cmd.execute();
    assertEquals(List.of(new Decisions.Outcome("note", 1, "right", false)), outcomes);
    cmd.execute();
    assertTrue(cmd.isFinished());
    cmd.end(false);
  }

  @Test
  public void testMarkerLocksChoiceEarly() throws Exception {
    AtomicInteger seen = new AtomicInteger(1);
    List<String> namesSeenByDecider = new ArrayList<>();
    Decisions.register(
        "note",
        ctx -> {
          namesSeenByDecider.addAll(ctx.optionNames());
          return seen.get();
        });
    Command cmd = parse(decisionJson(1.0, 0));

    // The marker command runs on the path before the node and locks in option 1
    Command marker = NamedCommands.getCommand("decide:note");
    marker.initialize();
    marker.end(false);
    assertEquals(List.of("left", "right"), namesSeenByDecider);

    // What the decider sees later no longer matters
    seen.set(0);
    cmd.initialize();
    assertEquals(List.of(new Decisions.Outcome("note", 1, "right", false)), outcomes);

    // A marker for the next visit, fired while this option runs, survives end()
    marker.initialize();
    cmd.end(false);
    seen.set(1);
    outcomes.clear();
    cmd.initialize();
    assertEquals(0, outcomes.get(0).optionIndex());
    cmd.end(false);

    // The lock is used once
    outcomes.clear();
    cmd.initialize();
    assertEquals(1, outcomes.get(0).optionIndex());
  }

  @Test
  public void testTimeoutUsesDefault() throws Exception {
    Decisions.register("note", ctx -> -1);
    Command cmd = parse(decisionJson(0.5, 1));

    cmd.initialize();
    SimHooks.stepTiming(0.3);
    cmd.execute();
    assertTrue(outcomes.isEmpty());

    SimHooks.stepTiming(0.3);
    cmd.execute();
    assertEquals(List.of(new Decisions.Outcome("note", 1, "right", true)), outcomes);
  }

  @Test
  public void testRequirementsAreUnion() {
    Subsystem a = new Subsystem() {};
    Subsystem b = new Subsystem() {};
    DecisionCommand cmd =
        new DecisionCommand(
            "union", 1.0, 0, List.of("a", "b"), List.of(Commands.idle(a), Commands.idle(b)));
    assertEquals(java.util.Set.of(a, b), cmd.getRequirements());
  }
}
