cmake_minimum_required(VERSION 3.16)
project(nimbro_primitive_fitter)

SET(CMAKE_VERBOSE_MAKEFILE TRUE)
SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake) # For finding custom .cmake files

find_package(Boost REQUIRED COMPONENTS system regex)
find_package(Eigen3 REQUIRED)
find_package(PugiXML REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(roboptim-capsule REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
    include
    contrib/vcglib/
    contrib/approx_mvbb/include/ApproxMVBB
    contrib/approx_mvbb/include/ApproxMVBB/Config
    contrib/approx_mvbb/include/ApproxMVBB/Common
    contrib/approx_mvbb/external/Diameter/include/ApproxMVBB/Diameter
    contrib/approx_mvbb/external/Diameter/include/ApproxMVBB/Diameter/Utils
    contrib/approx_mvbb/external/GeometryPredicates/include/ApproxMVBB/GeometryPredicates
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PUGIXML_INCLUDE_DIR}
    ${roboptim-capsule_INCLUDE_DIRS}
)

link_directories(
    ${roboptim-capsule_LIBRARY_DIRS}
)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nimbro_primitive_fitter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

add_library(approx_mvbb
    contrib/approx_mvbb/src/ApproxMVBB/AABB.cpp
    contrib/approx_mvbb/src/ApproxMVBB/ConvexHull2D.cpp
    contrib/approx_mvbb/src/ApproxMVBB/MinAreaRectangle.cpp
    contrib/approx_mvbb/src/ApproxMVBB/OOBB.cpp
    contrib/approx_mvbb/src/ApproxMVBB/RandomGenerators.cpp
    contrib/approx_mvbb/src/ApproxMVBB/Common/MyMatrixTypeDefs.cpp
    contrib/approx_mvbb/external/GeometryPredicates/src/PredicatesInit.cpp
    contrib/approx_mvbb/external/GeometryPredicates/src/Predicates.cpp
    contrib/approx_mvbb/external/Diameter/src/rand.cpp
    contrib/approx_mvbb/external/Diameter/src/alloc.cpp
    contrib/approx_mvbb/external/Diameter/src/util.cpp
    contrib/approx_mvbb/external/Diameter/src/EstimateDiameter.cpp
)

add_executable(nimbro_primitive_fitter
    src/primitive_fitter.cpp
    src/inertia.cpp
    src/util.cpp
    src/main.cpp
)

target_link_libraries(nimbro_primitive_fitter
    rclcpp::rclcpp
    ${PUGIXML_LIBRARIES}
    roboptim-capsule::roboptim-capsule
    approx_mvbb
  )


string(REPLACE ";" " " WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS_LIST}")

set(CMAKE_CXX_FLAGS "${WARNING_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
install(TARGETS
  nimbro_primitive_fitter
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  approx_mvbb
  DESTINATION lib
)

install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
