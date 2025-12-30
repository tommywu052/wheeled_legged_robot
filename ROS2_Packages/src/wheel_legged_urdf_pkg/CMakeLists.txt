cmake_minimum_required(VERSION 3.8)
project(wheel_legged_urdf_pkg)

# 查找 ament_cmake 构建工具
find_package(ament_cmake REQUIRED)

# 安装目录（config、launch、meshes、urdf）
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 必须调用此宏来生成必要的CMake配置文件
ament_package()