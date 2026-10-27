cmake_minimum_required(VERSION 3.5)
project(obs_fitting)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  visualization_msgs
  cv_bridge
)

# set(PCL_DIR "/usr/local/pcl-1.13.1/share/pcl-1.13")
# find_package(PCL 1.13 REQUIRED COMPONENTS)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINATIONS})

# 自动检测CUDA
execute_process(
    COMMAND bash -c "dpkg -l | grep cuda-core | sed -e 's/ \\{1,\\}/ /g' | cut -d ' ' -f 3 | cut -d '.' -f 1,2 | sed -e 's/-.*//g' | sort -n | tail -n 1"
    OUTPUT_VARIABLE DETECTED_CUDA_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(DETECTED_CUDA_VERSION)
    set(CUDAVERSION "cuda-${DETECTED_CUDA_VERSION}")
    set(CUDA_DIR "/usr/local/${CUDAVERSION}")
else()
    set(CUDAVERSION "cuda")
    set(CUDA_DIR "/usr/local/cuda")
endif()

if(NOT EXISTS "${CUDA_DIR}")
    message(FATAL_ERROR "CUDA not found at ${CUDA_DIR}")
endif()

message(STATUS "Using CUDA directory: ${CUDA_DIR}")

find_package(CUDA REQUIRED)

# +++ 直接指定 cudaFilter 和 cudaCluster 库路径 +++
set(CUDA_FILTER_LIB "/home/gcs/kitti_test/src/obs_fitting/include/lib/libcudafilter.so")
set(CUDA_CLUSTER_LIB "/home/gcs/kitti_test/src/obs_fitting/include/lib/libcudacluster.so")

# 检查cudaFilter库
if(NOT EXISTS ${CUDA_FILTER_LIB})
    message(FATAL_ERROR "cudaFilter library not found at: ${CUDA_FILTER_LIB}")
else()
    message(STATUS "Using cudaFilter library: ${CUDA_FILTER_LIB}")
    get_filename_component(CUDA_FILTER_LIB_DIR ${CUDA_FILTER_LIB} DIRECTORY)
    message(STATUS "cudaFilter library directory: ${CUDA_FILTER_LIB_DIR}")
endif()

# 检查cudaCluster库
if(NOT EXISTS ${CUDA_CLUSTER_LIB})
    message(WARNING "cudaCluster library not found at: ${CUDA_CLUSTER_LIB}")
    message(WARNING "Will try to find it in standard library paths")
    
    # 尝试在标准路径中查找
    find_library(CUDA_CLUSTER_LIB 
        NAMES cudacluster cudaCluster
        PATHS 
            /usr/local/lib
            /usr/lib
            ${CUDA_DIR}/lib64
            /home/gcs/kitti_test/src/obs_fitting/src/lib
    )
    
    if(CUDA_CLUSTER_LIB)
        message(STATUS "Found cudaCluster library: ${CUDA_CLUSTER_LIB}")
    else()
        message(FATAL_ERROR "cudaCluster library not found. Please build it first.")
    endif()
else()
    message(STATUS "Using cudaCluster library: ${CUDA_CLUSTER_LIB}")
endif()

# 获取库文件目录
get_filename_component(CUDA_CLUSTER_LIB_DIR ${CUDA_CLUSTER_LIB} DIRECTORY)

# 设置链接器标志
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined -pthread")

find_package(OpenCV REQUIRED)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)

# 设置PCL相关库
set(PCL_LIBRARIES_EXTRA
    pcl_common pcl_io pcl_recognition pcl_features 
    pcl_sample_consensus pcl_octree pcl_search 
    pcl_filters pcl_kdtree pcl_segmentation 
    pcl_visualization
)

find_package(Boost REQUIRED COMPONENTS system)

###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES patchworkpp_test
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${CUDA_DIR}/include
    /usr/local/include
    /usr/include/eigen3
    # /usr/include/pcl-1.10
    /usr/include/vtk-6.3
    ${Eigen3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    # 添加包含 cudaFilter.h 和 cudaCluster.h 的目录
    ${PROJECT_SOURCE_DIR}/include
    # /home/gcs/kitti_test/src/patchworkpp_test/src
    # /home/gcs/kitti_test/src/patchworkpp_test/src/lib  # 添加lib目录
)

# 添加链接目录
link_directories(
    ${CUDA_LIBRARY_DIRS}
    ${CUDA_DIR}/lib64
    # /usr/lib
    # /usr/local/lib
    # /usr/lib/aarch64-linux-gnu
    ${PCL_LIBRARY_DIRS}
    # 添加包含 cudaFilter 和 cudaCluster 库的目录
    # /home/gcs/kitti_test/src/patchworkpp_test/src/lib
    ${CUDA_FILTER_LIB_DIR}
    ${CUDA_CLUSTER_LIB_DIR}
)

file(GLOB CPP_SOURCES "src/*.cpp")
message(STATUS "Found CPP sources: ${CPP_SOURCES}")

add_executable(main ${CPP_SOURCES})

## +++ 关键：链接 cudaFilter 和 cudaCluster 库 +++
target_link_libraries(main
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PCL_LIBRARIES_EXTRA}
    ${OpenCV_LIBS}
    ${OpenMP_CXX_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUDA_CUDART_LIBRARY}
    ${Boost_LIBRARIES}
    # 链接 cudaFilter 和 cudaCluster 共享库
    ${CUDA_FILTER_LIB}
    ${CUDA_CLUSTER_LIB}
    # CUDA 相关库
    cudart
    # 系统库
    rt
    dl
    pthread
    stdc++fs
)

# +++ 重要：设置运行时库搜索路径 +++
target_link_options(main PRIVATE 
    "SHELL:-Wl,-rpath,${CUDA_FILTER_LIB_DIR}"
    "SHELL:-Wl,-rpath,${CUDA_CLUSTER_LIB_DIR}"
    "SHELL:-Wl,-rpath,${CUDA_DIR}/lib64"
)

set_target_properties(main PROPERTIES
    CXX_STANDARD 14
    POSITION_INDEPENDENT_CODE ON
)

#############
## Install ##
#############