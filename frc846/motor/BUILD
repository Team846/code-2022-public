cc_library(
    name = "motor",
    srcs = [
        "config.cc",
        "gains.cc",
        "helper.cc",
    ],
    hdrs = [
        "config.h",
        "converter.h",
        "gains.h",
        "helper.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc846",
        "@bazelrio//libraries/cpp/ctre/phoenix",
        "@bazelrio//libraries/cpp/rev/revlib",
    ],
)

cc_test(
    name = "converter_test",
    size = "small",
    srcs = [
        "converter_test.cc",
    ],
    deps = [
        "//frc846/motor",
        "@com_google_googletest//:gtest_main",
    ],
)
