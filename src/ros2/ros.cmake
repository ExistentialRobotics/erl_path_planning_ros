message(STATUS "ROS2 activated, building ROS2 stuff")

file(GLOB_RECURSE ROS2_SOURCES src/ros2/*.cpp)

foreach(src_file IN LISTS ROS2_SOURCES)
    get_filename_component(name ${src_file} NAME_WE)
    add_executable(${name} ${src_file})
    erl_target_dependencies(${name})
    erl_collect_targets(EXECUTABLES ${name})
endforeach()
