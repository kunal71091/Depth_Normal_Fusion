project(vision-normal-fusion)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-Wall -O3 -std=c++11")

include_directories(
    "libs"
    "libs/D3D/src"
)

# Link libraries
add_library(d3d
"libs/D3D/src/d3d_base/cameraMatrix.cpp"
"libs/D3D/src/d3d_base/colorMapJet.cpp"
"libs/D3D/src/d3d_base/configFile.cpp"
"libs/D3D/src/d3d_base/depthMap.cpp"
"libs/D3D/src/d3d_base/grid.cpp"
"libs/D3D/src/d3d_fusion/volumetricFusionTools.cpp"
"libs/D3D/src/d3d_io/ioTools.cpp"
)


# OpenCV
find_package( OpenCV REQUIRED )
if(OpenCV_FOUND)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Boost
# set(Boost_DEBUG ON)
find_package( Boost COMPONENTS program_options system filesystem REQUIRED )
if(Boost_FOUND)
include_directories( ${Boost_INCLUDE_DIRS} )

add_executable(tvHistDepthIntegrator src/tvHistDepthIntegrator.cpp)
target_link_libraries(tvHistDepthIntegrator ${Boost_LIBRARIES} ${OpenCV_LIBS} d3d)

add_executable(tvHistNormalIntegrator src/tvHistNormalIntegrator.cpp)
target_link_libraries(tvHistNormalIntegrator ${Boost_LIBRARIES} ${OpenCV_LIBS} d3d)

add_executable(tvHistNormalClustering src/tvHistNormalClustering.cpp)
target_link_libraries(tvHistNormalClustering ${Boost_LIBRARIES} d3d)

add_executable(tvHistFusion src/tvHistFusion.cpp)
target_link_libraries(tvHistFusion ${Boost_LIBRARIES} d3d)
set_source_files_properties(src/tvHistFusion.cpp PROPERTIES COMPILE_FLAGS "-fopenmp")
set_target_properties(tvHistFusion PROPERTIES LINK_FLAGS "-fopenmp")

add_executable(tvHistNormalFusion src/tvHistNormalFusion.cpp)
target_link_libraries(tvHistNormalFusion ${Boost_PROGRAM_OPTIONS_LIBRARY} d3d)
set_source_files_properties(src/tvHistNormalFusion.cpp PROPERTIES COMPILE_FLAGS "-fopenmp")
set_target_properties(tvHistNormalFusion PROPERTIES LINK_FLAGS "-fopenmp")

endif(Boost_FOUND)
endif(OpenCV_FOUND)

