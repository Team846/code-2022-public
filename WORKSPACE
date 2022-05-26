load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

### bazelrio

# Using local bazel
# local_repository(
#     name = "bazelrio",
#     path = "../bazelrio/bazelrio",
# )

# Make sure to update the checksum and strip_prefix when updating release
# http_archive(
#     name = "bazelrio",
#     sha256 = "ba3359f9273515f3a0695555dbacbba118c39c18fe37187f5ea84f158de864cb",
#     strip_prefix = "bazelRio-22.2.1/bazelrio",
#     url = "https://github.com/Team846/bazelRio/archive/refs/tags/v22.2.1.zip",
# )

http_archive(
    name = "bazelrio",
    url = "https://github.com/bazelRio/bazelRio/archive/refs/tags/0.10.0.zip",
    sha256 = "18b109dbd5204910600823e6c9ff405fa7ed7c43d0a78f24077f8187311745a9",
    strip_prefix = "bazelRio-0.10.0/bazelrio",
)

load("@bazelrio//:deps.bzl", "setup_bazelrio_dependencies")

setup_bazelrio_dependencies()

load("@bazelrio//:defs.bzl", "setup_bazelrio")

setup_bazelrio()

### compile commands generator

http_archive(
    name = "hedron_compile_commands",
    sha256 = "9fda864ddae428ad0e03f7669ff4451554afac116e913cd2cd619ac0f40db8d9",
    strip_prefix = "bazel-compile-commands-extractor-1d21dc390e20ecb24d73e9dbb439e971e0d30337",
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/1d21dc390e20ecb24d73e9dbb439e971e0d30337.tar.gz",
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()

### gtest

http_archive(
    name = "com_google_googletest",
    sha256 = "1cff5915c9dfbf8241d811e95230833c4f34a6d56b7b8c960f4c828f60429a38",
    strip_prefix = "googletest-c9461a9b55ba954df0489bab6420eb297bed846b",
    urls = ["https://github.com/google/googletest/archive/c9461a9b55ba954df0489bab6420eb297bed846b.zip"],
)
