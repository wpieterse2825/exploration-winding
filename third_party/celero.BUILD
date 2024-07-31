cc_library(
    name = "celero",
    srcs = glob(
        include = [
            "**/*.cpp",
        ],
        exclude = [
            "test/**",
            "experiments/**",
        ],
    ),
    hdrs = glob(
        include = [
            "**/*.h",
        ],
        exclude = [
            "test/**",
            "experiments/**",
        ],
    ),
    strip_include_prefix = "include",
    visibility = [
        "//visibility:public",
    ],
)
