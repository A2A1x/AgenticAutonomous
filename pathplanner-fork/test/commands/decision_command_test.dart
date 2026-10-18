import 'package:file/memory.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/auto/pathplanner_auto.dart';
import 'package:pathplanner/commands/command.dart';
import 'package:pathplanner/commands/command_groups.dart';
import 'package:pathplanner/commands/decision_command.dart';
import 'package:pathplanner/commands/named_command.dart';
import 'package:pathplanner/commands/path_command.dart';
import 'package:pathplanner/commands/wait_command.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:pathplanner/widgets/editor/tree_widgets/commands/command_group_widget.dart';
import 'package:undo/undo.dart';

DecisionCommand _decision() => DecisionCommand(
      name: 'note',
      timeout: 1.5,
      defaultOption: 1,
      options: [
        DecisionOption(name: 'left', commands: [
          PathCommand(pathName: 'Left'),
          NamedCommand(name: 'intake'),
        ]),
        DecisionOption(name: 'right', commands: [
          SequentialCommandGroup(commands: [PathCommand(pathName: 'Right')]),
          WaitCommand(waitTime: 0.2),
        ]),
      ],
    );

PathPlannerAuto _auto(List<Command> commands) => PathPlannerAuto(
      name: 'test',
      autoDir: '/autos',
      fs: MemoryFileSystem(),
      sequence: SequentialCommandGroup(commands: commands),
      resetOdom: true,
      folder: null,
      choreoAuto: false,
    );

({Pose2d start, Pose2d end}) _ends(num x0, num y0, num x1, num y1,
        [num endDeg = 0]) =>
    (
      start: Pose2d(Translation2d(x0, y0), const Rotation2d()),
      end: Pose2d(Translation2d(x1, y1), Rotation2d.fromDegrees(endDeg)),
    );

void main() {
  test('toJson/fromJson round trip', () {
    DecisionCommand cmd = _decision();
    Map<String, dynamic> json = cmd.toJson();

    expect(json['type'], 'decision');
    expect(json['data']['name'], 'note');
    expect(json['data']['timeout'], 1.5);
    expect(json['data']['defaultOption'], 1);
    expect(json['data']['options'][0]['name'], 'left');
    expect(json['data']['options'][0]['commands'][0]['type'], 'path');

    expect(Command.fromJson(json), cmd);
  });

  test('clone is deep', () {
    DecisionCommand cmd = _decision();
    DecisionCommand cloned = cmd.clone();
    expect(cloned, cmd);

    cloned.options[0].name = 'changed';
    cloned.options[1].commands.clear();
    expect(cmd.options[0].name, 'left');
    expect(cmd.options[1].commands, isNotEmpty);
  });

  test('auto paths follow the default option', () {
    PathPlannerAuto auto = _auto([PathCommand(pathName: 'Start'), _decision()]);

    expect(auto.getAllPathNames(), ['Start', 'Right']);
    expect(auto.getAllPathNames(allOptions: true), ['Start', 'Left', 'Right']);

    auto.updatePathName('Left', 'Left2');
    expect(auto.getAllPathNames(allOptions: true), ['Start', 'Left2', 'Right']);
  });

  test('decision warnings', () {
    final ends = {
      'Start': _ends(0, 0, 5, 5),
      'Left': _ends(5, 5, 8, 1),
      'Right': _ends(5, 5, 8, 1),
      'Far': _ends(1, 1, 8, 1),
      'Turned': _ends(5, 5, 8, 1, 90),
    };

    // Lined up and nothing after: no warnings
    expect(
        _auto([PathCommand(pathName: 'Start'), _decision()])
            .getDecisionWarnings(ends),
        isEmpty);

    // Option starts away from where the robot arrives
    DecisionCommand d = _decision();
    d.options[0].commands[0] = PathCommand(pathName: 'Far');
    expect(
        _auto([PathCommand(pathName: 'Start'), d]).getDecisionWarnings(ends), [
      'Decision "note": option "left" does not start where the robot arrives'
    ]);

    // Options end in different rotations: only matters with a command after
    d = _decision();
    d.options[0].commands[0] = PathCommand(pathName: 'Turned');
    expect(_auto([PathCommand(pathName: 'Start'), d]).getDecisionWarnings(ends),
        isEmpty);
    expect(
        _auto([PathCommand(pathName: 'Start'), d, NamedCommand(name: 'shoot')])
            .getDecisionWarnings(ends),
        [
          'Decision "note": option "left" does not end where the default option ends'
        ]);
  });

  testWidgets('add from menu, edit options, undo', (widgetTester) async {
    SequentialCommandGroup sequence = SequentialCommandGroup(commands: []);
    ChangeStack undoStack = ChangeStack();
    await widgetTester.pumpWidget(StatefulBuilder(
      builder: (context, setState) => MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: CommandGroupWidget(
              command: sequence,
              undoStack: undoStack,
              onUpdated: () => setState(() {}),
              allPathNames: const ['path1'],
            ),
          ),
        ),
      ),
    ));

    await widgetTester.tap(find.byTooltip('Add Command'));
    await widgetTester.pumpAndSettle();
    await widgetTester.tap(find.text('Decision'));
    await widgetTester.pumpAndSettle();
    DecisionCommand decision = sequence.commands.single as DecisionCommand;
    expect(find.text('Decision:'), findsOneWidget);

    await widgetTester.tap(find.byTooltip('Add Option'));
    await widgetTester.pumpAndSettle();
    expect(decision.options.map((o) => o.name),
        ['Option 1', 'Option 2', 'Option 3']);

    await widgetTester.tap(find.byIcon(Icons.star_border).last);
    await widgetTester.pumpAndSettle();
    expect(decision.defaultOption, 2);

    undoStack.undo();
    undoStack.undo();
    await widgetTester.pumpAndSettle();
    expect(decision.options.length, 2);
    expect(decision.defaultOption, 0);
  });
}
