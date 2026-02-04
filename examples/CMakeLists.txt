set(SOURCE_FILES
    paths.h
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