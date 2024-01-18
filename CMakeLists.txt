cmake_minimum_required(VERSION 3.5)
project(lanelet2_divider)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(GeographicLib REQUIRED)
find_package(lanelet2 REQUIRED)
find_package(GDAL REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include
        ${lanelet2_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS})

set(LANELET2_DIVIDER_LIB_SRC
        src/lanelet2_divider.cpp
        src/mgrs_projector.cpp)

set(LANELET2_DIVIDER_LIB_HEADERS
        include/lanelet2_divider/lanelet2_divider.hpp
        include/lanelet2_divider/mgrs_projector.hpp)


ament_auto_add_library(${PROJECT_NAME} SHARED
        ${LANELET2_DIVIDER_LIB_HEADERS}
        ${LANELET2_DIVIDER_LIB_SRC})
target_link_libraries(${PROJECT_NAME}
        ${lanelet2_LIBRARIES}
        ${GDAL_LIBRARIES}
        ${GeographicLib_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES})

#target_compile_options(${PROJECT_NAME} PRIVATE
#  -Wall
#  -Werror
#  -Wextra
#  #-Wshadow             # causes issues with ROS 2 headers
#  #-Wnon-virtual-dtor   # causes issues with ROS 2 headers
#  -pedantic
#  -Wcast-align
#  -Wunused
#  -Wconversion
#  -Wsign-conversion
#  -Wdouble-promotion
#  #-Wnull-dereference    # gcc6
#  #-Wduplicated-branches # gcc7
#  #-Wduplicated-cond     # gcc6
#  #-Wrestrict            # gcc7
#  -fvisibility=hidden)
# C++-only options

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "lanelet2_divider::Lanelet2Divider"
        EXECUTABLE ${PROJECT_NAME}_exe)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

#install(DIRECTORY
#        launch
#        DESTINATION share/${PROJECT_NAME}/
#)
#install(DIRECTORY
#        params
#        DESTINATION share/${PROJECT_NAME}
#)

ament_export_include_directories(include)
ament_export_libraries({${PROJECT_NAME}_node})

ament_auto_package()