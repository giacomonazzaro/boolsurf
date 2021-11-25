add_executable(mesh-vector-graphics mesh-vector-graphics.cpp app.h render.h spline_editing.h utils.h)

set_target_properties(mesh-vector-graphics  PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(mesh-vector-graphics  PRIVATE ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(mesh-vector-graphics  yocto boolsurf)

if(YOCTO_OPENGL)
target_link_libraries(mesh-vector-graphics  yocto_gui)
endif(YOCTO_OPENGL)
