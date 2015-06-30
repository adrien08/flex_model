cmake_minimum_required(VERSION 2.8.3)
project(flex_models)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE Release)

find_package(Eigen 3 REQUIRED)
include_directories(include
    ${EIGEN_INCLUDE_DIRS}
)

add_library(flex_model SHARED
   src/flex_model.cpp
   src/flex_model_12.cpp
   src/flex_model_23.cpp
   src/flex_model_42.cpp
)

add_executable(command_corr_kuka_flex src/main.cpp)
target_link_libraries(command_corr_kuka_flex flex_model)

## Mark executables and/or libraries for installation
# install(TARGETS polaris_sensor polaris_sensor_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
