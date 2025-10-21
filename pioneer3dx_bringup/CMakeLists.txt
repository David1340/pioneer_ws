cmake_minimum_required(VERSION 3.5)
project(pioneer3dx_bringup)

# Encontra pacotes necessários
find_package(ament_cmake REQUIRED)

# Instala arquivos de launch
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# Instala arquivos de configuração (yaml, urdf, etc.) se houver
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Instala package.xml (necessário para ROS 2)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Declara o pacote
ament_package()
