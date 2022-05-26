cc_library(
    name = "frc846",
    srcs = [
        "math.cc",
        "named.cc",
        "sendable_callback.cc",
        "swerve_odometry.cc",
        "trajectory_generator.cc",
        "xbox.cc",
    ],
    hdrs = [
        "ctre_namespace.h",
        "grapher.h",
        "math.h",
        "named.h",
        "pref.h",
        "sendable_callback.h",
        "subsystem.h",
        "swerve_odometry.h",
        "trajectory_generator.h",
        "xbox.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@bazelrio//libraries/cpp/ctre/phoenix",
        "@bazelrio//libraries/cpp/rev/revlib",
        "@bazelrio//libraries/cpp/wpilib/new_commands",
        "@bazelrio//libraries/cpp/wpilib/wpilibc",
    ],
)

cc_test(
    name = "grapher_test",
    size = "small",
    srcs = [
        "grapher_test.cc",
    ],
    deps = [
        ":frc846",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "math_test",
    size = "small",
    srcs = [
        "math_test.cc",
    ],
    deps = [
        ":frc846",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "named_test",
    size = "small",
    srcs = [
        "named_test.cc",
    ],
    deps = [
        ":frc846",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pref_test",
    size = "small",
    srcs = [
        "pref_test.cc",
    ],
    deps = [
        ":frc846",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "subsystem_test",
    size = "small",
    srcs = [
        "subsystem_test.cc",
    ],
    deps = [
        ":frc846",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "trajectory_generator_test",
    size = "small",
    srcs = [
        "trajectory_generator_test.cc",
    ],
    deps = [
        ":frc846",
        "@com_google_googletest//:gtest_main",
    ],
)
