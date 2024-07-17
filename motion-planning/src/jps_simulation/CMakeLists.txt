cmake_minimum_required(VERSION 2.8.3)
project(jps_simulation)
set(my_cv_bridge_DIR "/home/lemontech/thirdlib/cv_bridge_ws/devel/share/my_cv_bridge/cmake")
set(OpenCV_DIR "/usr/local/opencv-4.6.0/bulid/")

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    pcl_ros
    geometry_msgs
    sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable ( jps_simulation_node 
    
    src/get_laser_map.cpp

    src/Node_class/Node.cpp 
    src/pcl2grid_class/pcl2grid.cpp       # 将pcl点云地图转换为二维栅格地图
    src/Jps_class/Jps.cpp                # 将pcl2grid中获取到的栅格数据全部封装进jps类，以便后续进行路径规划等操作
    src/publisher/publisher.cpp   #定义话题发布器
    # src/turtle_robot.cpp
    )
target_link_libraries( jps_simulation_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} )  