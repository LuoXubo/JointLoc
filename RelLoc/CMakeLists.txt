# Redis version

cmake_minimum_required(VERSION 2.8.3)
project(demo)

add_compile_options(-fPIC)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})


SET(CMAKE_CXX_FLAGS "-std=c++17 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
add_definitions(-w)

# Opencv
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 REQUIRED)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

# Eigen, Ceres and Pangolin
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)


# Protobuf
find_package(Protobuf REQUIRED)
    if(NOT PROTOBUF_FOUND)
        message(FATAL_ERROR "Protobuf not found")
    endif()
include_directories(
    ${PROTOBUF_INCLUDE_DIR}
    )

file(GLOB PROTO_FILES ${PROJECT_SOURCE_DIR}/protobuf/*.proto)
message("PROTO_FILES: ${PROTO_FILES}")
foreach(proto_name ${PROTO_FILES})
      file(RELATIVE_PATH proto_rel_name ${PROJECT_SOURCE_DIR} ${proto_name})
      list(APPEND REL_PROTO_FILES ${proto_rel_name})
endforeach()
message("REL_PROTO_FILES: ${REL_PROTO_FILES}")

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS  ${PROJECT_SOURCE_DIR}/${REL_PROTO_FILES})
message("PROTO_SRCS: ${PROTO_SRCS}")
message("PROTO_HDRS: ${PROTO_HDRS}")
add_library(proto_lib STATIC ${PROTO_SRCS} ${PROTO_HDRS})
message("PROTOBUF_INCLUDE_DIR: ${PROTOBUF_INCLUDE_DIR}")
target_include_directories(proto_lib PUBLIC ${PROTOBUF_INCLUDE_DIR})
target_link_libraries(proto_lib PRIVATE ${PROTOBUF_STATIC_LIBRARIES})

add_library(${PROJECT_NAME} 
    ${PROTO_SRCS}
    ${PROTO_HDRS}
    )

target_link_libraries(
    ${PROJECT_NAME}
    ${PROTOBUF_LIBRARY}
    )

# Redis
find_package(Threads)
find_library(HIREDIS_LIB hiredis)
find_library(REDIS_PLUS_PLUS_LIB redis++)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR}) 

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/modules)

include_directories(./ ./src/)
include_directories(./ ./3rdParty/)

add_subdirectory(3rdParty)

set(SLAM_SRC_FILES
    src/slam/Converter.cc
    src/slam/Frame.cc
    src/slam/FrameDrawer.cc
    src/slam/System.cc       
    src/slam/Sim3Solver.cc
    src/slam/Tracking.cc
    src/slam/LocalMapping.cc 
    src/slam/ORBextractor.cc 
    src/slam/PnPsolver.cc
    src/slam/ORBmatcher.cc
    src/slam/MapPoint.cc
    src/slam/KeyFrame.cc
    src/slam/Map.cc
    src/slam/MapDrawer.cc
    src/slam/PosePredictor.cc
    src/slam/Initializer.cc
    src/slam/ModelBA.cc     
    src/slam/KeyFrameDatabase.cc
    src/slam/Viewer.cc
    src/slam/loadArg.cc
    src/multiView/robustPoseEstimate.cpp 
    src/multiView/multiViewCompute.cpp
)

set(OPENGMVG_SOURCE_FILES
    src/OpenMVG/multiview/conditioning.cpp
    src/OpenMVG/multiview/essential.cpp
    src/OpenMVG/multiview/projection.cpp
    src/OpenMVG/multiview/solver_affine.cpp
    src/OpenMVG/multiview/solver_essential_five_point.cpp
    src/OpenMVG/multiview/solver_essential_kernel.cpp
    src/OpenMVG/multiview/solver_fundamental_kernel.cpp
    src/OpenMVG/multiview/solver_homography_kernel.cpp
    src/OpenMVG/multiview/solver_resection_kernel.cpp
    src/OpenMVG/multiview/triangulation.cpp
    src/OpenMVG/multiview/triangulation_nview.cpp
    src/OpenMVG/robust_estimation/guided_matching.hpp
    src/OpenMVG/numeric/numeric.cpp
    src/OpenMVG/geometry/rigid_transformation3D_srt.cpp
    src/OpenMVG/geometry/Similarity3.cpp
    src/OpenMVG/geometry/Similarity3_Kernel.cpp
)


set(THIRD_LIBS  stlplus DBoW2 ${CERES_LIBRARIES} ${Pangolin_LIBRARIES})


SET(SRC_LIST src/ReTMatch.cc)
add_executable(ReTMacher ${SRC_LIST})
target_link_libraries(ReTMacher ${OpenCV_LIBS} ${THIRD_LIBS})				  

add_executable(main_BowCvt src/tools/BowCvt.cc)
target_link_libraries(main_BowCvt ${THIRD_LIBS})	

set(Glog_DIR "/usr/local/lib/cmake/glog/")

add_executable(jointslam src/vslam.cc ${SLAM_SRC_FILES} ${OPENGMVG_SOURCE_FILES})
target_link_libraries(jointslam ${PROJECT_NAME})
target_link_libraries(jointslam  ${THIRD_LIBS}  -lpthread -lm )
target_link_libraries(jointslam ${REDIS_PLUS_PLUS_LIB} pthread ${HIREDIS_LIB})
