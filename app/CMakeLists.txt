set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../stark/extern/Eigen)
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
set (CMAKE_CXX_FLAGS "-DEIGEN_MPL2_ONLY")

set(SOURCE_FILES
    scenes.cpp
    scenes.h
    main.cpp
	)

# Create executable    
add_executable(app ${SOURCE_FILES})

# Include/link dependencies
## Stark
target_link_libraries(app stark)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../stark/include)

## Eigen
target_include_directories(app PUBLIC ${EIGEN3_INCLUDE_DIR})