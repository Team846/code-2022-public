load("@bazelrio//:defs.bzl", "robot_cc_binary")

cc_library(
    name = "ports",
    hdrs = [
        "ports.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "field",
    hdrs = [
        "field.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["//frc846"],
)

robot_cc_binary(
    name = "robot",
    srcs = [
        "funky_robot.cc",
        "funky_robot.h",
        "main.cc",
    ],
    halsim_configs = {
        "gui": ["@bazelrio//libraries/cpp/wpilib/simulation/halsim_gui"],
    },
    linkopts = ["-ldl"],
    team_number = 846,
    deps = [
        "//cameraserver",
        "//frc846",
        "//robot/autos",
        "//robot/commands",
        "//robot/subsystems",
    ],
)
