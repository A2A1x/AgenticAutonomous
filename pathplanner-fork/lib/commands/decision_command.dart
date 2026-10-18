import 'package:flutter/foundation.dart';
import 'package:pathplanner/commands/command.dart';
import 'package:pathplanner/commands/command_groups.dart';

/// One choice of a decision: a named list of commands run in sequence.
class DecisionOption extends SequentialCommandGroup {
  String name;

  DecisionOption({required this.name, required super.commands});

  DecisionOption.fromDataJson(super.json)
      : name = json['name'] ?? 'Option',
        super.fromDataJson();

  @override
  Map<String, dynamic> dataToJson() {
    return {
      'name': name,
      ...super.dataToJson(),
    };
  }

  @override
  DecisionOption clone() {
    return DecisionOption(
        name: name, commands: CommandGroup.cloneCommandsList(commands));
  }

  @override
  bool operator ==(Object other) =>
      other is DecisionOption &&
      other.name == name &&
      listEquals(other.commands, commands);

  @override
  int get hashCode => Object.hash(name, commands);
}

/// Picks one of its options on the robot at runtime. The options live in
/// [commands], so the generic command tree walks (path renames, named
/// commands, missing paths) reach every option.
class DecisionCommand extends CommandGroup {
  String name;
  num timeout;
  int defaultOption;

  DecisionCommand({
    this.name = 'Decision',
    this.timeout = 1.0,
    this.defaultOption = 0,
    List<DecisionOption>? options,
  }) : super(
          type: 'decision',
          commands: options ??
              [
                DecisionOption(name: 'Option 1', commands: []),
                DecisionOption(name: 'Option 2', commands: []),
              ],
        );

  DecisionCommand.fromDataJson(Map<String, dynamic> json)
      : name = json['name'] ?? 'Decision',
        timeout = json['timeout'] ?? 1.0,
        defaultOption = json['defaultOption'] ?? 0,
        super(type: 'decision', commands: [
          for (Map<String, dynamic> optionJson in (json['options'] ?? []))
            DecisionOption.fromDataJson(optionJson),
        ]);

  List<DecisionOption> get options => commands.cast<DecisionOption>();

  /// The commands of the default option, which the auto's path list,
  /// starting pose and simulation follow.
  List<Command> get defaultOptionCommands => options.isEmpty
      ? []
      : options[defaultOption.clamp(0, options.length - 1)].commands;

  @override
  Map<String, dynamic> dataToJson() {
    return {
      'name': name,
      'timeout': timeout,
      'defaultOption': defaultOption,
      'options': [
        for (DecisionOption option in options) option.dataToJson(),
      ],
    };
  }

  @override
  DecisionCommand clone() {
    return DecisionCommand(
      name: name,
      timeout: timeout,
      defaultOption: defaultOption,
      options: [for (DecisionOption option in options) option.clone()],
    );
  }

  @override
  bool operator ==(Object other) =>
      other is DecisionCommand &&
      other.name == name &&
      other.timeout == timeout &&
      other.defaultOption == defaultOption &&
      listEquals(other.commands, commands);

  @override
  int get hashCode => Object.hash(type, name, timeout, defaultOption, commands);
}
