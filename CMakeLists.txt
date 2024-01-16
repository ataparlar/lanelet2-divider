cmake_minimum_required(VERSION 3.8)
project(lanelet2_divider)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lanelet2 REQUIRED)
find_package(GDAL REQUIRED)


include_directories(include
        ${rclcpp_INCLUDE_DIRS}
        ${lanelet2_INCLUDE_DIRS}
)

# Lanelet2 Divider
add_executable(lanelet2_divider_exe
        src/lanelet2_divider.cpp)
ament_target_dependencies(lanelet2_divider_exe rclcpp lanelet2)
target_link_libraries(lanelet2_divider_exe
        ${lanelet2_LIBRARIES}
        GDAL::GDAL)


install(TARGETS lanelet2_divider_exe
        DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY launch
#        DESTINATION share/${PROJECT_NAME}/)
#install(DIRECTORY config
#        DESTINATION share/${PROJECT_NAME}/)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
