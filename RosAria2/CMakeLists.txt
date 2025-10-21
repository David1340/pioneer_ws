cmake_minimum_required(VERSION 3.5)
project(rosaria)

# Suporte a C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Diretórios do Aria - ajustar conforme sua instalação
set(ARIA_INCLUDE_DIR "/usr/local/include/Aria")
set(ARIA_LIBRARY_DIR "/usr/local/lib")

# Mensagens de debug
message(STATUS "Using Aria include dir: ${ARIA_INCLUDE_DIR}")
message(STATUS "Using Aria library dir: ${ARIA_LIBRARY_DIR}")

# Encontrar dependências ROS 2 e Boost
find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# Suporte a mensagens customizadas
find_package(rosidl_default_generators REQUIRED)

# Geração da interface da mensagem
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BumperState.msg"
  DEPENDENCIES std_msgs
)

# Diretórios de includes
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${ARIA_INCLUDE_DIR}
)

link_directories(${ARIA_LIBRARY_DIR})

# Executável principal
add_executable(RosAria
  RosAria.cpp
  LaserPublisher.cpp
)

# Dependências do executável
ament_target_dependencies(RosAria
  Boost
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

# Associar interfaces de mensagem ao executável
rosidl_target_interfaces(RosAria
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Link de bibliotecas adicionais
target_link_libraries(RosAria
  ${Boost_LIBRARIES}
  Aria
  pthread
  dl
  rt
)

# Instalação
install(TARGETS RosAria
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
