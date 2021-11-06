add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  sensor_msgs
  tf2_ros
  geometry_msgs
  nav_msgs
  urdf)
find_package(choreonoid REQUIRED)

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

include_directories(${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
rtmbuild_add_library(CommandLevelROSBridge SHARED CommandLevelROSBridge.cpp)
target_link_libraries(CommandLevelROSBridge ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES_ABS})
set_target_properties(CommandLevelROSBridge PROPERTIES PREFIX "") # libCommandLevelROSBridge.so -> CommandLevelROSBridge.so
add_dependencies(CommandLevelROSBridge RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(CommandLevelROSBridgeComp CommandLevelROSBridgeComp.cpp)
target_link_libraries(CommandLevelROSBridgeComp CommandLevelROSBridge)
