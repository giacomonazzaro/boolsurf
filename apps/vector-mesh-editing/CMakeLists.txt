add_executable(app  app.cpp)

set_target_properties(app  PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(app  PRIVATE ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(app  yocto)

if(YOCTO_OPENGL)
target_link_libraries(app  yocto_gui)
endif(YOCTO_OPENGL)
