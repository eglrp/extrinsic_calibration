
SET(execName checkerboard	)
cmake_minimum_required(VERSION 2.8)

project(${execName})
set(CMAKE_BUILD_TYPE Debug)
SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(ZED REQUIRED)
find_package(RPLidar REQUIRED)

message("")
message("-- Found OpenCV:\t ${OpenCV_VERSION}\t (${OpenCV_INCLUDE_DIRS})")
message("-- Found RPLidar:\t ${RPLidar_VERSION}\t (${RPLIDAR_INCLUDE_DIR})")
message("-- Found ZED:\t\t ${ZED_VERSION}\t (${ZED_INCLUDE_DIRS})")
message("")

find_package(Ceres REQUIRED)


include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${RPLIDAR_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CERES_LIBRARIES_DIRS}) 
link_directories(${RPLIDAR_LIBRARY_DIR})


SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")

SET(HEADER_FOLDER include)
FILE(GLOB_RECURSE HEADER_FILES "${HEADER_FOLDER}/*.hpp" "${HEADER_FOLDER}/*.h")

#unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
#option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
#message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
add_definitions(-std=c++11 -g) #We added this******

add_executable(${execName} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${execName}     
			${CERES_LIBRARIES} 
		    ${ZED_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${LINUX_LIBRARIES}
            ${CUDA_CUDA_LIBRARY}
            ${RPLIDAR_LIBRARY}
)


