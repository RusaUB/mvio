cmake_minimum_required(VERSION 3.0)
project(mvio)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(mvio src/main.cpp)

target_link_libraries(mvio 
    ${OpenCV_LIBS} 
    Eigen3::Eigen
)
