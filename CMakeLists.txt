cmake_minimum_required(VERSION 3.5)
project(co_lrio)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)

# Include directories
include_directories(
	include
  include/third_parties
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Defined message and service
set(msg_files
  msg/OptimizationRequest.msg
  msg/LoopClosure.msg
  msg/OptimizationResponse.msg
  srv/SaveFiles.srv
)

rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
  DEPENDENCIES std_msgs nav_msgs sensor_msgs geometry_msgs
)

# Maximum Clique Solver
add_library(fast_max_clique_finder
  include/third_parties/fast_max-clique_finder/src/findClique.h
  include/third_parties/fast_max-clique_finder/src/graphIO.h
  include/third_parties/fast_max-clique_finder/src/findClique.cpp
  include/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp
  include/third_parties/fast_max-clique_finder/src/utils.cpp
  include/third_parties/fast_max-clique_finder/src/graphIO.cpp
)
install(
  TARGETS fast_max_clique_finder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# nabo library
add_library(nabo_lib
  include/third_parties/nabo/nabo.h
  include/third_parties/nabo/nabo.cpp
  include/third_parties/nabo/nabo_private.h
  include/third_parties/nabo/index_heap.h
  include/third_parties/nabo/kdtree_opencl.cpp
  include/third_parties/nabo/brute_force_cpu.cpp
  include/third_parties/nabo/kdtree_cpu.cpp
)
install(
  TARGETS nabo_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# geo graphic library
add_library(geographic_lib
  include/third_parties/geo_graphic/Config.h
  include/third_parties/geo_graphic/Constants.hpp
  include/third_parties/geo_graphic/Geocentric.cpp
  include/third_parties/geo_graphic/Geocentric.hpp
  include/third_parties/geo_graphic/LocalCartesian.cpp
  include/third_parties/geo_graphic/LocalCartesian.hpp
  include/third_parties/geo_graphic/Math.cpp
  include/third_parties/geo_graphic/Math.hpp
)
install(
  TARGETS geographic_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# risam library
add_library(risam_lib
  include/third_parties/risam/DoglegLineSearch.h
  include/third_parties/risam/GraduatedFactor.cpp
  include/third_parties/risam/GraduatedFactor.h
  include/third_parties/risam/GraduatedKernel.cpp
  include/third_parties/risam/GraduatedKernel.h
  include/third_parties/risam/RISAM2.cpp
  include/third_parties/risam/RISAM2.h
)
target_link_libraries(risam_lib PRIVATE gtsam)
install(
  TARGETS risam_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# gicp library
add_library(gicp_lib
  include/third_parties/nabo/nabo.h
  include/third_parties/nabo/nabo.cpp
  include/third_parties/nabo/nabo_private.h
  include/third_parties/nabo/index_heap.h
  include/third_parties/nabo/kdtree_opencl.cpp
  include/third_parties/nabo/brute_force_cpu.cpp
  include/third_parties/nabo/kdtree_cpu.cpp

  include/third_parties/flann/flann.cpp
  include/third_parties/flann/flann.h

  include/third_parties/ikd_tree/ikd_Tree.h
  include/third_parties/ikd_tree/ikd_Tree.cpp

  include/third_parties/gicp/so3/so3.hpp
  include/third_parties/gicp/impl/fast_gicp_impl.hpp
  include/third_parties/gicp/impl/fast_vgicp_impl.hpp
  include/third_parties/gicp/impl/lsq_registration_impl.hpp
  include/third_parties/gicp/gicp_settings.hpp
  include/third_parties/gicp/fast_vgicp_voxel.hpp
  include/third_parties/gicp/fast_gicp.hpp
  include/third_parties/gicp/fast_gicp.cpp
 include/third_parties/gicp/fast_vgicp.hpp
 include/third_parties/gicp/fast_vgicp.cpp
  include/third_parties/gicp/lsq_registration.hpp
  include/third_parties/gicp/lsq_registration.cpp
)
install(
  TARGETS gicp_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Lidar odometry
add_executable(${PROJECT_NAME}_lidar_odometry src/scanContextDescriptor.cpp src/lidarIrisDescriptor.cpp  src/lidarOdometry.cpp)
ament_target_dependencies(${PROJECT_NAME}_lidar_odometry
  rclcpp rclpy
  std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs pcl_msgs
  tf2 tf2_ros tf2_geometry_msgs tf2_eigen tf2_sensor_msgs
  OpenCV PCL GTSAM Eigen)
target_link_libraries(${PROJECT_NAME}_lidar_odometry gtsam gtsam_unstable gicp_lib geographic_lib)
rosidl_target_interfaces(${PROJECT_NAME}_lidar_odometry ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Concentrated mapping
add_executable(${PROJECT_NAME}_concentrated_mapping src/scanContextDescriptor.cpp src/lidarIrisDescriptor.cpp src/concentratedMapping.cpp)
ament_target_dependencies(${PROJECT_NAME}_concentrated_mapping
  rclcpp rclpy
  std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs pcl_msgs
  tf2 tf2_ros tf2_geometry_msgs tf2_eigen tf2_sensor_msgs
  OpenCV PCL GTSAM Eigen)
target_link_libraries(${PROJECT_NAME}_concentrated_mapping gtsam risam_lib fast_max_clique_finder nabo_lib gicp_lib geographic_lib)
rosidl_target_interfaces(${PROJECT_NAME}_concentrated_mapping ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_lidar_odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_concentrated_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()