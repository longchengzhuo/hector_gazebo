cmake_minimum_required(VERSION 3.8)
project(hector_gazebo) # 项目名称已修改

find_package(ament_cmake REQUIRED)

ament_package()

# 安装 launch 目录，目标路径也已更新
install(DIRECTORY
        launch assets
        DESTINATION share/${PROJECT_NAME} # ${PROJECT_NAME} 会自动变为 hector_gazebo
)