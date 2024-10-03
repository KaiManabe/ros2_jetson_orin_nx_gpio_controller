cmake_minimum_required(VERSION 3.8)
project(jetsonorinnx_gpio_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED)
find_package(gpio_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

include_directories(include)

add_executable(jetsonorinnx_gpio_controller
                src/Main.cpp
                src/GpioControllerNode.cpp
                )

ament_target_dependencies(jetsonorinnx_gpio_controller rclcpp gpio_msgs)

install(TARGETS jetsonorinnx_gpio_controller
  DESTINATION lib/jetsonorinnx_gpio_controller
)


target_include_directories(jetsonorinnx_gpio_controller PRIVATE ${GPIOD_INCLUDE_DIRS})
target_link_libraries(jetsonorinnx_gpio_controller ${GPIOD_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
