cmake_minimum_required(VERSION 3.12)
project(point_cloud_tool)

# Set cpp standard
set(CMAKE_CXX_STANDARD 14)

# PCL library
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Qt library
set(CMAKE_PREFIX_PATH "/usr/local/opt/Qt5.9.7/5.9.7/clang_64/lib/cmake/Qt5")
# find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR on)
# instruct cmake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# create code from a list of qt designer ui files
set(CMAKE_AUTOUIC ON)
# resource
set(CMAKE_AUTORCC ON)
# find qtwidgets library
find_package(Qt5 REQUIRED Widgets)
find_package(VTK REQUIRED)

# compile & link
set(SOURCE_FILES main.cpp main_window.cpp tool/triangulation.cpp pojo/MyPointCloud.cpp pojo/MyPolygonMesh.cpp main_window.ui main_window.qrc tool/projection.cpp tool/projection.h tool/resampling.cpp tool/resampling.h tool/filter.cpp tool/filter.h tool/io.cpp tool/io.h tool/utils.cpp tool/utils.h pojo/MyException.cpp pojo/MyException.h)
add_executable(${PROJECT_NAME} MACOSX_BUNDLE ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} )
qt5_use_modules(${PROJECT_NAME} Widgets)
