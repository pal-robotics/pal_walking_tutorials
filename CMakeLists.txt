cmake_minimum_required(VERSION 2.8.3)
project(pal_walking_tutorials)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(CATKIN_COMPONENTS
  pal_walking_msgs
  gazebo_msgs
  actionlib
  humanoid_nav_msgs
  eigen_checks)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES link_states_listener
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(link_states_listener src/link_states_listener.cpp)
target_link_libraries(link_states_listener ${catkin_LIBRARIES})

add_executable(walk_client_example_topic src/walk_client_example_topic.cpp)
target_link_libraries(walk_client_example_topic  ${catkin_LIBRARIES})

add_executable(walk_client_example_service src/walk_client_example_service.cpp)
target_link_libraries(walk_client_example_service  ${catkin_LIBRARIES})

add_executable(walk_client_example_action src/walk_client_example_action.cpp)
target_link_libraries(walk_client_example_action  ${catkin_LIBRARIES})

#add_executable(walk_safety_example src/safety_example.cpp)
#target_link_libraries(walk_safety_example  ${catkin_LIBRARIES})

#add_executable(walk_ik_example src/ik_example.cpp)
#target_link_libraries(walk_ik_example  ${catkin_LIBRARIES})

#############
## Install ##
#############
install(TARGETS walk_client_example_topic
                walk_client_example_service
                walk_client_example_action
                #walk_safety_example
                #walk_ik_example
                link_states_listener
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


