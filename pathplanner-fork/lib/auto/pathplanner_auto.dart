import 'dart:convert';

import 'package:file/file.dart';
import 'package:path/path.dart';
import 'package:pathplanner/commands/named_command.dart';
import 'package:pathplanner/commands/command.dart';
import 'package:pathplanner/commands/command_groups.dart';
import 'package:pathplanner/commands/decision_command.dart';
import 'package:pathplanner/commands/path_command.dart';
import 'package:pathplanner/pages/project/project_page.dart';
import 'package:pathplanner/services/log.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';

const String fileVersion = '2025.0';

class PathPlannerAuto {
  String name;
  SequentialCommandGroup sequence;
  bool resetOdom;
  bool choreoAuto;

  String? folder;

  FileSystem fs;
  String autoDir;

  // Stuff used for UI
  DateTime lastModified = DateTime.now().toUtc();

  PathPlannerAuto({
    required this.name,
    required this.sequence,
    required this.resetOdom,
    required this.autoDir,
    required this.fs,
    required this.folder,
    required this.choreoAuto,
  }) {
    _addNamedCommandsToEvents(sequence.commands);
  }

  PathPlannerAuto.defaultAuto({
    this.name = 'New Auto',
    this.resetOdom = true,
    required this.autoDir,
    required this.fs,
    this.folder,
    this.choreoAuto = false,
  }) : sequence = SequentialCommandGroup(commands: []);

  PathPlannerAuto duplicate(String newName) {
    return PathPlannerAuto(
      name: newName,
      sequence: sequence.clone() as SequentialCommandGroup,
      resetOdom: resetOdom,
      autoDir: autoDir,
      fs: fs,
      folder: folder,
      choreoAuto: choreoAuto,
    );
  }

  PathPlannerAuto.fromJson(
      Map<String, dynamic> json, String name, String autosDir, FileSystem fs)
      : this(
          autoDir: autosDir,
          fs: fs,
          name: name,
          sequence:
              Command.fromJson(json['command'] ?? {}) as SequentialCommandGroup,
          resetOdom: json['resetOdom'] ?? true,
          folder: json['folder'],
          choreoAuto: json['choreoAuto'] ?? false,
        );

  Map<String, dynamic> toJson() {
    return {
      'version': fileVersion,
      'command': sequence.toJson(),
      'resetOdom': resetOdom,
      'folder': folder,
      'choreoAuto': choreoAuto,
    };
  }

  static Future<List<PathPlannerAuto>> loadAllAutosInDir(
      String autosDir, FileSystem fs) async {
    List<PathPlannerAuto> autos = [];

    List<FileSystemEntity> files = fs.directory(autosDir).listSync();
    for (FileSystemEntity e in files) {
      if (e.path.endsWith('.auto')) {
        final file = fs.file(e.path);
        String jsonStr = await file.readAsString();
        try {
          Map<String, dynamic> json = jsonDecode(jsonStr);
          String autoName = basenameWithoutExtension(e.path);

          PathPlannerAuto auto =
              PathPlannerAuto.fromJson(json, autoName, autosDir, fs);
          auto.lastModified = (await file.lastModified()).toUtc();

          if (json['version'] != fileVersion) {
            auto.saveFile();
          }

          autos.add(auto);
        } catch (ex, stack) {
          Log.error('Failed to load auto', ex, stack);
        }
      }
    }
    return autos;
  }

  void rename(String name) {
    File autoFile = fs.file(join(autoDir, '${this.name}.auto'));

    if (autoFile.existsSync()) {
      autoFile.rename(join(autoDir, '$name.auto'));
    }
    this.name = name;
    lastModified = DateTime.now().toUtc();
  }

  void delete() {
    File autoFile = fs.file(join(autoDir, '$name.auto'));

    if (autoFile.existsSync()) {
      autoFile.delete();
    }
  }

  void saveFile() {
    try {
      File autoFile = fs.file(join(autoDir, '$name.auto'));
      const JsonEncoder encoder = JsonEncoder.withIndent('  ');
      autoFile.writeAsString(encoder.convert(this));
      lastModified = DateTime.now().toUtc();
      Log.debug('Saved "$name.auto"');
    } catch (ex, stack) {
      Log.error('Failed to save auto', ex, stack);
    }
  }

  void updatePathName(String oldPathName, String newPathName) {
    _updatePathNameInCommands(sequence.commands, oldPathName, newPathName);
    saveFile();
  }

  void _updatePathNameInCommands(
      List<Command> commands, String oldPathName, String newPathName) {
    for (Command cmd in commands) {
      if (cmd is PathCommand && cmd.pathName == oldPathName) {
        cmd.pathName = newPathName;
      } else if (cmd is CommandGroup) {
        _updatePathNameInCommands(cmd.commands, oldPathName, newPathName);
      }
    }
  }

  void _addNamedCommandsToEvents(List<Command> commands) {
    for (Command cmd in commands) {
      if (cmd is NamedCommand) {
        if (cmd.name != null) {
          ProjectPage.events.add(cmd.name!);
          continue;
        }
      }

      if (cmd is DecisionCommand) {
        ProjectPage.events.add('decide:${cmd.name}');
      }

      if (cmd is CommandGroup) {
        _addNamedCommandsToEvents(cmd.commands);
      }
    }
  }

  /// Paths in the order they run, following each decision's default option.
  /// With [allOptions], paths in every decision option are included too.
  List<String> getAllPathNames({bool allOptions = false}) {
    return _getPathNamesInCommands(sequence.commands, allOptions);
  }

  /// Warnings for decision options that don't line up with the rest of the
  /// auto. [ends] maps path names to the poses where they start and end.
  List<String> getDecisionWarnings(
      Map<String, ({Pose2d start, Pose2d end})> ends) {
    List<String> warnings = [];
    Pose2d? lastEnd;
    // Decisions (with their arrival pose) not yet followed by another command
    List<(DecisionCommand, Pose2d?)> unfollowed = [];

    Pose2d? endOf(List<Command> cmds, Pose2d? arrival) {
      final names = _getPathNamesInCommands(cmds, false);
      return names.isEmpty ? arrival : ends[names.last]?.end;
    }

    void walk(List<Command> commands) {
      for (Command cmd in commands) {
        // Options followed by more commands must all end at the same pose
        for (final (decision, arrival) in unfollowed) {
          Pose2d? defaultEnd = endOf(decision.defaultOptionCommands, arrival);
          for (DecisionOption option in decision.options) {
            Pose2d? end = endOf(option.commands, arrival);
            if (defaultEnd != null &&
                end != null &&
                !_samePose(end, defaultEnd)) {
              warnings.add(
                  'Decision "${decision.name}": option "${option.name}" does not end where the default option ends');
            }
          }
        }
        unfollowed.clear();

        if (cmd is PathCommand) {
          lastEnd = ends[cmd.pathName]?.end ?? lastEnd;
        } else if (cmd is DecisionCommand) {
          // With no path before it, the robot starts at the default option's first path
          final defaultNames =
              _getPathNamesInCommands(cmd.defaultOptionCommands, false);
          Pose2d? arrival = lastEnd ??
              (defaultNames.isEmpty ? null : ends[defaultNames.first]?.start);
          for (DecisionOption option in cmd.options) {
            final names = _getPathNamesInCommands(option.commands, false);
            Pose2d? start = names.isEmpty ? null : ends[names.first]?.start;
            if (arrival != null &&
                start != null &&
                start.translation.getDistance(arrival.translation) >= 0.25) {
              warnings.add(
                  'Decision "${cmd.name}": option "${option.name}" does not start where the robot arrives');
            }
          }
          walk(cmd.defaultOptionCommands);
          unfollowed.add((cmd, arrival));
        } else if (cmd is CommandGroup) {
          walk(cmd.commands);
        }
      }
    }

    walk(sequence.commands);
    return warnings;
  }

  static bool _samePose(Pose2d a, Pose2d b) {
    return a.translation.getDistance(b.translation) < 0.25 &&
        (a.rotation - b.rotation).degrees.abs() < 10;
  }

  bool hasEmptyPathCommands() {
    return _hasEmptyPathCommands(sequence.commands);
  }

  bool _hasEmptyPathCommands(List<Command> commands) {
    for (Command cmd in commands) {
      if (cmd is PathCommand && cmd.pathName == null) {
        return true;
      } else if (cmd is CommandGroup) {
        bool hasEmpty = _hasEmptyPathCommands(cmd.commands);
        if (hasEmpty) {
          return true;
        }
      }
    }
    return false;
  }

  bool hasEmptyNamedCommand() {
    return _hasEmptyNamedCommand(sequence.commands);
  }

  bool _hasEmptyNamedCommand(List<Command> commands) {
    for (Command cmd in commands) {
      if (cmd is NamedCommand && cmd.name == null) {
        return true;
      } else if (cmd is CommandGroup) {
        bool hasEmpty = _hasEmptyNamedCommand(cmd.commands);
        if (hasEmpty) {
          return true;
        }
      }
    }
    return false;
  }

  List<String> _getPathNamesInCommands(
      List<Command> commands, bool allOptions) {
    List<String> names = [];
    for (Command cmd in commands) {
      if (cmd is PathCommand && cmd.pathName != null) {
        names.add(cmd.pathName!);
      } else if (cmd is DecisionCommand && !allOptions) {
        names.addAll(
            _getPathNamesInCommands(cmd.defaultOptionCommands, allOptions));
      } else if (cmd is CommandGroup) {
        names.addAll(_getPathNamesInCommands(cmd.commands, allOptions));
      }
    }
    return names;
  }

  void handleMissingPaths(List<String> pathNames) {
    return _handleMissingPaths(sequence.commands, pathNames);
  }

  void _handleMissingPaths(List<Command> commands, List<String> pathNames) {
    for (Command cmd in commands) {
      if (cmd is PathCommand && !pathNames.contains(cmd.pathName)) {
        cmd.pathName = null;
      } else if (cmd is CommandGroup) {
        _handleMissingPaths(cmd.commands, pathNames);
      }
    }
  }

  @override
  bool operator ==(Object other) =>
      other is PathPlannerAuto &&
      other.runtimeType == runtimeType &&
      other.name == name &&
      other.sequence == sequence &&
      other.resetOdom == resetOdom;

  @override
  int get hashCode => Object.hash(name, sequence, resetOdom);
}
