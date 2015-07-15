cmake_minimum_required(VERSION 2.8.3)
project(lwr_flex_models)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED)
find_package(Eigen 3 REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
)

include_directories(include
    ${EIGEN_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/flex_model.cpp
   src/flex_model_12.cpp
   src/flex_model_23.cpp
   src/flex_model_42.cpp
)

add_executable(command_corr_kuka_flex src/main.cpp)
target_link_libraries(command_corr_kuka_flex ${PROJECT_NAME})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
