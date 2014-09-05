cmake_minimum_required(VERSION 2.8)
project(PointContour)

find_package(OPENGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLFW REQUIRED)
find_package(GLUT REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_search_module(GLFW REQUIRED glfw3)

#set(CMAKE_C_COMPILER "/Users/Winmad/llvm/build/Debug+Asserts/bin/clang2")
set(CMAKE_CXX_COMPILER "/Users/Winmad/llvm/build/Debug+Asserts/bin/clang2++")
#set(CMAKE_CXX_COMPILER "/opt/local/bin/g++")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -std=c++11")
set(CXX_DEBUG -O2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_DEBUG} -fopenmp -stdlib=libc++ -I/Users/Winmad/wxWidgets-3.0.1/build-osx/lib/wx/include/osx_cocoa-unicode-static-3.0 -I/Users/Winmad/wxWidgets-3.0.1/include")
set(CMAKE_LINK_FLAGS "-framework IOKit -framework Carbon -framework Cocoa -framework AudioToolbox -framework System -framework OpenGL -framework AGL -lpng -lz -ljpeg -ltiff -framework WebKit -lwxregexu-3.0 -lz -lpthread -liconv")

set(OPENMP_INCLUDE_DIRS "~/Intel-OpenMP/libomp_oss/exports/common/include")
set(OPENMP_LIBRARY_DIRS "~/Intel-OpenMP/libomp_oss/exports/mac_32e/lib.thin/")

add_definitions(-D_FILE_OFFSET_BITS=64 -D__WXMAC__ -D__WXOSX__ -D__WXOSX_COCOA__)

include_directories(
  ${OPENGL_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${OPENMP_INCLUDE_DIRS}
  ${wxWidgets_USE_FILE}
  include
  src
  GUI
  external/include
  external/lib/wx/wx/include/osx_cocoa-unicode-static-3.0
)

link_directories(
  ${GLFW_LIBRARY_DIRS}
  ${OPENGL_LIBRARY_DIRS}
  ${GLUT_LIBRARY_DIRS}
  ${GLEW_LIBRARY_DIRS}
  ${OPENMP_LIBRARY_DIRS}
  /Users/Winmad/wxWidgets-3.0.1/build-osx/lib 
)

set(ALL_LIBS
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLFW_LIBRARIES}
  libwx_osx_cocoau-3.0.a
  libwx_osx_cocoau_gl-3.0.a
)

AUX_SOURCE_DIRECTORY(src/ CPP_SOURCES)
AUX_SOURCE_DIRECTORY(GUI/ GUI_SOURCES)
set(CPP_SOURCES ${CPP_SOURCES} ${GUI_SOURCES})
add_executable(PointContour ${CPP_SOURCES})
set_target_properties(PointContour
  PROPERTIES LINK_FLAGS ${CMAKE_LINK_FLAGS}
)
target_link_libraries(PointContour ${ALL_LIBS})

#add_executable(canvas src/wxCanvasTest.cpp)
#set_target_properties(canvas
#  PROPERTIES LINK_FLAGS ${CMAKE_LINK_FLAGS}
#)
#target_link_libraries(canvas ${ALL_LIBS})
