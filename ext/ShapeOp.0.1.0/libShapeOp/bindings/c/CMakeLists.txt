include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories (${PROJECT_BINARY_DIR}) 
add_executable (runmeC runme.c)
target_link_libraries (runmeC ShapeOp)