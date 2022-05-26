cc_library(
    name = "subsystems",
    srcs = [
        "climber.cc",
        "climber_arm.cc",
        "driver.cc",
        "drivetrain.cc",
        "feeder.cc",
        "intake.cc",
        "leds.cc",
        "limelight.cc",
        "operator.cc",
        "shooter.cc",
        "swerve_module.cc",
    ],
    hdrs = [
        "climber.h",
        "climber_arm.h",
        "driver.h",
        "drivetrain.h",
        "feeder.h",
        "intake.h",
        "leds.h",
        "limelight.h",
        "operator.h",
        "robot_container.h",
        "shooter.h",
        "swerve_module.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc846",
        "//frc846/motor",
        "//frc846/wpilib",
        "//robot:field",
        "//robot:ports",
        "@bazelrio//libraries/cpp/ctre/phoenix",
        "@bazelrio//libraries/cpp/kauailabs/navx",
        "@bazelrio//libraries/cpp/wpilib/new_commands",
        "@bazelrio//libraries/cpp/wpilib/wpilibc",
    ],
)

cc_test(
    name = "drivetrain_test",
    size = "small",
    srcs = [
        "drivetrain_test.cc",
    ],
    linkstatic = 1,
    deps = [
        ":subsystems",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "swerve_module_test",
    size = "small",
    srcs = [
        "swerve_module_test.cc",
    ],
    linkstatic = 1,
    deps = [
        ":subsystems",
        "@com_google_googletest//:gtest_main",
    ],
)
