# Local maven repo: PathPlannerLib with decision nodes

`com.pathplanner.lib:PathplannerLib-java:2026.1.2-decisions`

Built from the PathPlanner fork in `../../pathplanner-fork` (upstream `v2026.1.2` plus the decision
changes, tracked as plain files in this repo). It is upstream PathPlannerLib 2026.1.2 plus `"decision"` nodes in
`.auto` files. Java only: there are no JNI or native parts, and the C++ and Python libs
can't parse decision nodes.

## Using it from the robot project

1. Delete `vendordeps/PathplannerLib.json` if the project has it. Gradle ranks `2026.1.2`
   above `2026.1.2-decisions`, so with both present it quietly picks upstream and the
   decision nodes do nothing.
2. In the robot's `build.gradle`, point a repository at this folder (the path is relative
   to the robot project, so adjust it) and add the dependency:

```groovy
repositories {
    maven { url = uri("$rootDir/../libs/maven") }
}

dependencies {
    implementation 'com.pathplanner.lib:PathplannerLib-java:2026.1.2-decisions'
}
```

3. In robot code, register deciders **before building autos**, the same way you register
   named commands:

```java
Decisions.register("centerNote", ctx -> {
    // ctx.optionNames() follow the order drawn in the app; ctx.secondsSinceReached() is
    // 0 when a decide:centerNote marker asks early
    if (vision.seesNote(3)) return 0;
    if (vision.seesNote(4)) return 1;
    return -1; // not decided yet: asked again next cycle until the node's timeout
});
PathPlannerLogging.setLogDecisionCallback(o ->
    Logger.recordOutput("Auto/Decision/" + o.decisionName(), o.optionName() + (o.timedOut() ? " (timeout)" : "")));
```

`Decisions.register` also registers the named command `decide:centerNote`. Put it as an
event marker on the path before the decision node so the choice is locked in while the
robot is still driving. With no marker, the robot polls the decider when it reaches the node
and gets no drive commands while it waits. If the path before the node ends with a non-zero
end velocity, the drivetrain keeps its last output during that wait. Give that path a zero
end velocity, or use the marker.

## Rebuilding

From `pathplanner-fork/pathplannerlib`, using the WPILib 2026 JDK:

```bash
JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" ./gradlew test publishJavaPublicationToMavenRepository -PreleaseMode -PpubVersion=2026.1.2-decisions -PrepoDir=../../libs/maven
```

`test` compiles the native test harness once (it needs the Visual Studio C++ tools) and runs
the Java tests, including `DecisionCommandTest`. Gradle doesn't cache artifacts from file
repositories, so republishing under the same version is picked up on the next robot build.
Use a new `-PpubVersion` if you want to keep the old jar too.
