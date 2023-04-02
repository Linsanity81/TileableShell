include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories (${PROJECT_BINARY_DIR}) 
add_executable (runmeCPP runme.cpp)
target_link_libraries (runmeCPP ShapeOp)