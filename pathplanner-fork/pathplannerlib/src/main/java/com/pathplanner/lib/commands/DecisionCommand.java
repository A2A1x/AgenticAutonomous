package com.pathplanner.lib.commands;

import com.pathplanner.lib.auto.Decisions;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.List;

/**
 * Runs one of several pre-built options, chosen by the decider registered with {@link
 * Decisions#register} under this decision's name. A choice locked in earlier by a
 * decide:&lt;name&gt; event marker is used if there is one. Otherwise the decider is polled every
 * cycle until it picks or the timeout passes, then the default option is used.
 */
public class DecisionCommand extends Command {
  private final String decisionName;
  private final double timeoutSeconds;
  private final int defaultOption;
  private final List<String> optionNames;
  private final List<Command> options;
  private final Timer timer = new Timer();

  private Command selected = null;

  /**
   * Create a decision command
   *
   * @param decisionName The decision name, used to find the decider
   * @param timeoutSeconds How long to wait for the decider after this command starts
   * @param defaultOption Index of the option to use on timeout. Clamped to a valid index
   * @param optionNames The option names
   * @param options The option commands, built ahead of time. Requirements are their union
   */
  public DecisionCommand(
      String decisionName,
      double timeoutSeconds,
      int defaultOption,
      List<String> optionNames,
      List<Command> options) {
    if (options.isEmpty() || options.size() != optionNames.size()) {
      throw new IllegalArgumentException(
          "A decision needs at least one option and one name per option");
    }
    this.decisionName = decisionName;
    this.timeoutSeconds = timeoutSeconds;
    this.defaultOption = MathUtil.clamp(defaultOption, 0, options.size() - 1);
    this.optionNames = List.copyOf(optionNames);
    this.options = List.copyOf(options);

    CommandScheduler.getInstance().registerComposedCommands(options.toArray(Command[]::new));
    for (Command option : options) {
      addRequirements(option.getRequirements());
    }
    Decisions.registerOptions(decisionName, this.optionNames);
    setName("Decision: " + decisionName);
  }

  @Override
  public void initialize() {
    timer.restart();
    selected = null;
    Integer locked = Decisions.takeLock(decisionName);
    if (locked != null) {
      select(locked, false);
    } else {
      poll();
    }
  }

  @Override
  public void execute() {
    if (selected == null) {
      poll();
    } else {
      selected.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return selected != null && selected.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (selected != null) {
      selected.end(interrupted);
    }
    timer.stop();
  }

  private void poll() {
    int choice = Decisions.ask(decisionName, new Decisions.Context(optionNames, timer.get()));
    if (choice >= 0) {
      select(choice, false);
    } else if (!Decisions.hasDecider(decisionName) || timer.hasElapsed(timeoutSeconds)) {
      select(defaultOption, true);
    }
  }

  private void select(int index, boolean timedOut) {
    selected = options.get(index);
    selected.initialize();
    PathPlannerLogging.logDecision(
        new Decisions.Outcome(decisionName, index, optionNames.get(index), timedOut));
  }
}
