cmake_minimum_required(VERSION 2.8.3)
project(pal_person_detector_opencv)

find_package(catkin REQUIRED COMPONENTS image_transport
                                        pal_detection_msgs
                                        cv_bridge
                                        roscpp
                                        geometry_msgs)
                                        
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(	
  CATKIN_DEPENDS image_transport cv_bridge pal_detection_msgs roscpp geometry_msgs
)

include_directories(include)

add_executable(pal_person_detector_opencv src/person_detector.cpp)
target_link_libraries(pal_person_detector_opencv ${catkin_LIBRARIES})

add_executable(tiago_move src/tiago_move.cpp)
target_link_libraries(tiago_move ${catkin_LIBRARIES})

install(TARGETS pal_person_detector_opencv
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
