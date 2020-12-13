# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;message_runtime;std_msgs;rbdl_model".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lrbdl_model_tests;-lrbdl_model".split(';') if "-lrbdl_model_tests;-lrbdl_model" != "" else []
PROJECT_NAME = "rbdl_model_tests"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
