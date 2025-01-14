cmake_minimum_required(VERSION 3.8)
project(ros2_odrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encuentra las dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_odrive_msgs REQUIRED)
#find_package(libusb-1.0 REQUIRED)

# Crea el ejecutable
add_executable(odrive
  src/odrive.cpp
  src/odrive_endpoint.cpp
  src/odrive_utils.cpp
)

# Declara las dependencias para el ejecutable
ament_target_dependencies(odrive
  rclcpp
  rclpy
  ros_odrive_msgs
)

# Incluye los directorios de encabezados
target_include_directories(odrive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Enlaza la biblioteca jsoncpp
target_link_libraries(odrive jsoncpp usb-1.0)

# Define las características del compilador
target_compile_features(odrive PUBLIC c_std_99 cxx_std_17)

# Instala el ejecutable
install(TARGETS odrive
  DESTINATION lib/${PROJECT_NAME}
)

# Opciones para pruebas
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Exporta el paquete
ament_package()
