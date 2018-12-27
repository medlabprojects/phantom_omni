cmake_minimum_required(VERSION 3.0)
project(phantom_omni)

add_definitions(-std=c++14) # use C++14

find_package(catkin
  REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    message_generation
    medlab_common
)

find_package(Qt5Core REQUIRED) # needed for signals/slots in PhantomOmniRos class
find_package(Eigen3 REQUIRED NO_MODULE)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   OmniState.msg
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
    include
    "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)

# make /devel/include directory if it doesn't exist yet
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

# create MOC code for Q_OBJECT classes
qt5_wrap_cpp(${PROJECT_NAME}_MOCS
  include/phantom_omni/phantom_omni_ros.h
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    medlab_common
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  /home/remireaa/Documents/LIBRARIES/MathTools
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/phantom_omni_ros.cpp
  ${${PROJECT_NAME}_MOCS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

