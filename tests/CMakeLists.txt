# Dependencies
add_subdirectory(extern/Catch2)

# Source files
set(SOURCE_FILES
    rb_constraints.cpp
	)

# Create executable    
add_executable(stark_tests ${SOURCE_FILES})

# Include/link dependencies
## Stark
target_link_libraries(stark_tests PRIVATE stark)
target_include_directories(stark_tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../stark/include)

## Catch2
target_link_libraries(stark_tests PRIVATE Catch2::Catch2WithMain)

# Define directory paths
target_compile_definitions(stark_tests PUBLIC STARK_TESTS_OUTPUT_DIR="${CMAKE_BINARY_DIR}/output")
