cmake_minimum_required(VERSION 2.8)
project(PointContour)

find_package(OPENGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLFW REQUIRED)
find_package(GLUT REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL 1.7 REQUIRED)

pkg_search_module(GLFW REQUIRED glfw3)

#set(CMAKE_C_COMPILER "/Users/Winmad/llvm/build/Debug+Asserts/bin/clang2")
set(CMAKE_CXX_COMPILER "/Users/Winmad/llvm/build/Debug+Asserts/bin/clang2++")
#set(CMAKE_CXX_COMPILER "/opt/local/bin/g++")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -std=c++11")
set(CXX_DEBUG -O2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_DEBUG} -fopenmp -stdlib=libc++ -Wno-ignored-attributes -Wno-deprecated-declarations")
#-I/Users/Winmad/wxWidgets-3.0.1/build-osx/lib/wx/include/osx_cocoa-unicode-static-3.0 -I/Users/Winmad/wxWidgets-3.0.1/include")
set(CMAKE_LINK_FLAGS "-framework IOKit -framework Carbon -framework Cocoa -framework AudioToolbox -framework System -framework OpenGL -framework AGL -lpng -lz -ljpeg -ltiff -framework WebKit -lwxregexu-3.0 -lz -lpthread -liconv -lmx -leng -lm -lmat -lut -lnlopt")
#-lmx -leng -lm -lmat -lut -lc++
#-lmwmclmcrrt -lmwmclmcr -lbs

set(OPENMP_INCLUDE_DIRS "~/Intel-OpenMP/libomp_oss/exports/common/include")
set(OPENMP_LIBRARY_DIRS "~/Intel-OpenMP/libomp_oss/exports/mac_32e/lib.thin/")

add_definitions(${PCL_DEFINITIONS} -D_FILE_OFFSET_BITS=64 -D__WXMAC__ -D__WXOSX__ -D__WXOSX_COCOA__)

include_directories(
  ${OPENGL_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${OPENMP_INCLUDE_DIRS}
  ${wxWidgets_USE_FILE}
  ${PCL_INCLUDE_DIRS}
  include
  coarseSuf/include
  sufTune/include
  src
  GUI
  external/include
  external/include/rapidxml
  external/tetgen/include
  external/lib/wx/wx/include/osx_cocoa-unicode-static-3.0
  /Applications/MATLAB_R2014a.app/extern/include
  /Users/Winmad/wxWidgets-3.0.1/build-osx/lib/wx/include/osx_cocoa-unicode-static-3.0
  /Users/Winmad/wxWidgets-3.0.1/include
)

link_directories(
  ${GLFW_LIBRARY_DIRS}
  ${OPENGL_LIBRARY_DIRS}
  ${GLUT_LIBRARY_DIRS}
  ${GLEW_LIBRARY_DIRS}
  ${OPENMP_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  /Users/Winmad/Projects/PointContour/external/tetgen/lib
  /Users/Winmad/Projects/PointContour/external/lib
  /Users/Winmad/wxWidgets-3.0.1/build-osx/lib
  /Applications/MATLAB_R2014a.app/bin/maci64
  /Applications/MATLAB_R2014a.app/runtime/maci64
  /Applications/MATLAB_R2014a.app/bin/maci64
  #/Applications/MATLAB/MATLAB_Compiler_Runtime/v83/runtime/maci64
  #/Applications/MATLAB/MATLAB_Compiler_Runtime/v83/bin/maci64
)

set(ALL_LIBS
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLFW_LIBRARIES}
  ${PCL_LIBRARIES}
  libwx_osx_cocoau-3.0.a
  libwx_osx_cocoau_gl-3.0.a
  libtet.a
  libsuf.a
  libsufTune.a
)

AUX_SOURCE_DIRECTORY(src/ CPP_SOURCES)
AUX_SOURCE_DIRECTORY(GUI/ GUI_SOURCES)

set(CPP_SOURCES ${CPP_SOURCES} ${GUI_SOURCES})
add_executable(PointContour ${CPP_SOURCES})
set_target_properties(PointContour
  PROPERTIES LINK_FLAGS ${CMAKE_LINK_FLAGS}
)
target_link_libraries(PointContour ${ALL_LIBS})

