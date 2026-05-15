set(SOURCE_FILES
    main.cpp
    rb_constraint_test_scenes.cpp
    rb_constraint_test_scenes.h
	)

# Create executable    
add_executable(examples ${SOURCE_FILES})

# Include/link dependencies
## Stark
target_link_libraries(examples stark)
target_include_directories(examples PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../stark/include)

# Define directory paths
target_compile_definitions(examples PUBLIC STARK_EXAMPLES_RES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/res")
target_compile_definitions(examples PUBLIC STARK_EXAMPLES_OUTPUT_DIR="${CMAKE_BINARY_DIR}/output")
