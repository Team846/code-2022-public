# code-2022-public

Snapshot of Team 846's robot code as of 5/26/22
 - Code run at Houston Championships
 - Merged in experimental shoot while driving branch
 - Minor cleanup and formatting

Features
 - Command based architecture with C++/bazel using [bazelrio](https://github.com/bazelRio/bazelRio/)
 - 5 ball autonomous routine
 - Custom swerve drive control, trajectory generator, and follower
 - Custom limelight python [pipeline](limelight.py) that fits an ellipse to the vision targets to aim correctly towards partially obscured targets
 - Hot configurable preferences
 - Motor controller wrappers to cache gains/configurations and reduce CAN usage
 
---

## Setup
1. install bazel
2. install clangd vscode extension (not microsoft c++ extension)
3. run `bazel run @hedron_compile_commands//:refresh_all` for c++ autocompletion
4. restart vscode

## Build for roborio
`bazel build //robot:robot.deploy --platforms=@bazelrio//platforms/roborio`

## Deploy to roborio
`bazel run //robot:robot.deploy --platforms=@bazelrio//platforms/roborio`

## Run desktop simulation
`bazel run //robot:robot.simulation.gui`

## Run unit tests
`bazel test --test_output=all //...`

