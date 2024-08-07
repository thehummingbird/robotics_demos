cmake_minimum_required(VERSION 2.8)
project( mono-vo )
find_package( OpenCV 4.0 REQUIRED )

include_directories( include ${OpenCV_INCLUDE_DIRS} )

add_library( visual_odometry SHARED src/vo.cpp)
add_executable( vo main.cpp )
target_link_libraries( vo visual_odometry ${OpenCV_LIBS} )
