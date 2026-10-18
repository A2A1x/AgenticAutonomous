import 'package:flutter/material.dart';
import 'package:pathplanner/commands/command.dart';
import 'package:pathplanner/commands/command_groups.dart';
import 'package:pathplanner/commands/decision_command.dart';
import 'package:pathplanner/pages/project/project_page.dart';
import 'package:pathplanner/widgets/editor/tree_widgets/commands/command_group_widget.dart';
import 'package:pathplanner/widgets/editor/tree_widgets/commands/duplicate_command_button.dart';
import 'package:pathplanner/widgets/number_text_field.dart';
import 'package:pathplanner/widgets/renamable_title.dart';
import 'package:undo/undo.dart';

class DecisionCommandWidget extends StatelessWidget {
  final DecisionCommand command;
  final VoidCallback? onUpdated;
  final VoidCallback? onRemoved;
  final double subCommandElevation;
  final List<String>? allPathNames;
  final ValueChanged<String?>? onPathCommandHovered;
  final ChangeStack undoStack;
  final VoidCallback? onDuplicateCommand;
  final bool showEditPathButton;
  final Function(String?)? onEditPathPressed;

  const DecisionCommandWidget({
    super.key,
    required this.command,
    this.onUpdated,
    this.onRemoved,
    this.subCommandElevation = 4.0,
    this.allPathNames,
    this.onPathCommandHovered,
    required this.undoStack,
    this.onDuplicateCommand,
    this.showEditPathButton = true,
    this.onEditPathPressed,
  });

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return Column(
      children: [
        Row(
          children: [
            const Text('Decision:', style: TextStyle(fontSize: 16)),
            Flexible(
              child: RenamableTitle(
                title: command.name,
                textStyle: const TextStyle(fontSize: 16),
                onRename: (value) => _change(command.name, (v) {
                  command.name = v;
                  ProjectPage.events.add('decide:$v');
                }, value),
              ),
            ),
            Expanded(child: Container()),
            Tooltip(
              message: 'Add Option',
              waitDuration: const Duration(milliseconds: 500),
              child: IconButton(
                onPressed: () => _changeOptions((options) => options.add(
                    DecisionOption(
                        name: 'Option ${options.length + 1}', commands: []))),
                icon: Icon(Icons.add, color: colorScheme.primary),
              ),
            ),
            Visibility(
              visible: onDuplicateCommand != null,
              child: DuplicateCommandButton(onPressed: onDuplicateCommand),
            ),
            Visibility(
              visible: onRemoved != null,
              child: Tooltip(
                message: 'Remove Command',
                waitDuration: const Duration(seconds: 1),
                child: IconButton(
                  onPressed: onRemoved,
                  visualDensity: const VisualDensity(
                      horizontal: VisualDensity.minimumDensity,
                      vertical: VisualDensity.minimumDensity),
                  icon: Icon(Icons.delete, color: colorScheme.error),
                ),
              ),
            ),
          ],
        ),
        const SizedBox(height: 8),
        NumberTextField(
          initialValue: command.timeout,
          label: 'Timeout (S)',
          minValue: 0.0,
          arrowKeyIncrement: 0.1,
          onSubmitted: (value) {
            if (value != null && value >= 0) {
              _change(command.timeout, (v) => command.timeout = v, value);
            }
          },
        ),
        for (int i = 0; i < command.options.length; i++)
          Card(
            elevation: subCommandElevation,
            color: colorScheme.surface,
            surfaceTintColor: colorScheme.surfaceTint,
            child: Padding(
              padding: const EdgeInsets.all(8.0),
              child: CommandGroupWidget(
                command: command.options[i],
                title: _buildOptionTitle(i),
                undoStack: undoStack,
                subCommandElevation: (subCommandElevation == 1.0) ? 4.0 : 1.0,
                onUpdated: onUpdated,
                onRemoved: command.options.length > 1
                    ? () => _changeOptions((options) {
                          options.removeAt(i);
                          if (command.defaultOption >= i &&
                              command.defaultOption > 0) {
                            command.defaultOption--;
                          }
                        })
                    : null,
                onDuplicateCommand: () => _changeOptions((options) {
                  DecisionOption copy = command.options[i].clone();
                  copy.name = '${copy.name} (copy)';
                  options.insert(i + 1, copy);
                }),
                allPathNames: allPathNames,
                onPathCommandHovered: onPathCommandHovered,
                showEditPathButton: showEditPathButton,
                onEditPathPressed: onEditPathPressed,
              ),
            ),
          ),
      ],
    );
  }

  Widget _buildOptionTitle(int optionIdx) {
    bool isDefault = command.defaultOption == optionIdx;

    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Tooltip(
          message: isDefault
              ? 'Default option: used on timeout, for the starting pose and the preview'
              : 'Make this the default option',
          waitDuration: const Duration(milliseconds: 500),
          child: IconButton(
            onPressed: () => _change(command.defaultOption,
                (v) => command.defaultOption = v, optionIdx),
            icon: Icon(isDefault ? Icons.star : Icons.star_border,
                color: isDefault ? Colors.amber : null),
          ),
        ),
        Flexible(
          child: RenamableTitle(
            title: command.options[optionIdx].name,
            textStyle: const TextStyle(fontSize: 16),
            onRename: (value) => _change(command.options[optionIdx].name,
                (v) => command.options[optionIdx].name = v, value),
          ),
        ),
      ],
    );
  }

  void _change<T>(T oldValue, void Function(T) set, T newValue) {
    undoStack.add(Change(
      oldValue,
      () {
        set(newValue);
        onUpdated?.call();
      },
      (old) {
        set(old);
        onUpdated?.call();
      },
    ));
  }

  void _changeOptions(void Function(List<Command> options) edit) {
    undoStack.add(Change(
      (CommandGroup.cloneCommandsList(command.commands), command.defaultOption),
      () {
        edit(command.commands);
        onUpdated?.call();
      },
      (old) {
        command.commands = CommandGroup.cloneCommandsList(old.$1);
        command.defaultOption = old.$2;
        onUpdated?.call();
      },
    ));
  }
}
