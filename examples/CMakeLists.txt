# Collect example files
file(GLOB examples "*.cpp")
list(SORT examples)
foreach(example_file ${examples})
	get_filename_component(example_base ${example_file} NAME_WE)
    message(STATUS "Adding ${example_base} example file")

    add_executable(${example_base} ${example_file})

    # Use find_package(DL) instead
    # ${DL_LIBRARIES}
    # See main CMakeLists.txt of cppadcodegen
    target_link_libraries(${example_base} Eigen3::Eigen -ldl)
endforeach(example_file)
