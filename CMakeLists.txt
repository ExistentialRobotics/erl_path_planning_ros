cmake_minimum_required(VERSION 3.16)

project(erl_path_planning_ros #
    LANGUAGES CXX #
    VERSION 0.1.0 #
    DESCRIPTION "erl_path_planning_ros is a ROS package of erl_path_planning")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if(NOT COMMAND erl_project_setup)
    find_package(erl_cmake_tools REQUIRED)
endif()
erl_project_setup(ERL_PACKAGES erl_path_planning)
erl_setup_ros(
    CATKIN_COMPONENTS roscpp geometry_msgs erl_geometry_msgs tf2_ros nav_msgs std_srvs
    CATKIN_DEPENDS roscpp geometry_msgs erl_geometry_msgs tf2_ros nav_msgs std_srvs
    ROS2_COMPONENTS rclcpp geometry_msgs erl_geometry_msgs tf2_ros nav_msgs std_srvs)
include_directories(include)
erl_add_ros_src()
erl_install(EXECUTABLES ${${PROJECT_NAME}_COLLECTED_EXECUTABLES})
erl_mark_project_found()
