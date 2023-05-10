set_languages("cxx17")
add_rules("mode.debug","mode.release")

target("trajectory_metrics")
    set_kind("binary")
    add_files("src/*.cpp")
