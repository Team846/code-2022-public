cc_library(
    name = "cameraserver",
    srcs =
        glob(["**/*.cpp"]),
    hdrs =
        glob([
            "**/*.h",
            "**/*.inc",
        ]),
    copts = ["-w"],
    visibility = ["//visibility:public"],
    deps = [
        "@bazelrio//libraries/cpp/wpilib/wpilibc",
    ],
)
