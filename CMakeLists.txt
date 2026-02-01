cmake_minimum_required(VERSION 3.15)
project(mvio VERSION 1.0.0 LANGUAGES CXX)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_library(mvio_lib
	src/sensor/FolderSource.cpp
	src/sensor/RpiSource.cpp
	src/sensor/PhoneSource.cpp
	src/vio/VisualOdometry.cpp
	src/vio/VisualOdometry3Dto2D.cpp
	src/vio/VisualInertialOdometry3Dto2D.cpp
	src/vio/ImuPreintegration.cpp
	src/estimators/KalmanFilter.cpp
	src/pipeline/Pipeline.cpp
)

add_library(mvio::lib ALIAS mvio_lib)

target_include_directories(mvio_lib
	PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(mvio_lib PUBLIC cxx_std_17)

target_link_libraries(mvio_lib
	PUBLIC
	${OpenCV_LIBS}
	Eigen3::Eigen
)

add_executable(mvio apps/app.cpp)

target_link_libraries(mvio
	PRIVATE
	mvio::lib
	${CMAKE_THREAD_LIBS_INIT}
)
