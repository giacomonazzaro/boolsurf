add_subdirectory(yocto)
add_subdirectory(boolsurf)
add_subdirectory(yocto_deprecated)

if(YOCTO_OPENGL)
add_subdirectory(yocto_gui)
endif(YOCTO_OPENGL)
