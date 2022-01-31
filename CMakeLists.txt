cmake_minimum_required(VERSION 2.6)
project(ICP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# Find Packages
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenMP)

include_directories(${EIGEN3_INCLUDE_DIR})

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
  add_compile_definitions(WITH_OPENMP) # preprocessor macro
endif()

include_directories(include)

add_executable(main
    src/main.cpp
)

if(OPENMP_FOUND)
    target_compile_options(main PRIVATE ${OpenMP_FLAGS})
    target_link_libraries(main ${OpenMP_LIBS})
endif()


if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )

  # For Use Not PreCompiled Features
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( main ${PCL_LIBRARIES} )
endif()
