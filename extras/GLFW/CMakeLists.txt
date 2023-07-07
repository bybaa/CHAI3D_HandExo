
cmake_minimum_required (VERSION 3.0.0)
project (GLFW)

# project version
set (PROJECT_VERSION 3.2)

# OpenGL dependency
find_package (OpenGL REQUIRED)
include_directories (OPENGL_INCLUDE_DIR)

# enforce build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release CACHE STRING "Setting build mode to Release" FORCE)
endif()


# 
# build products
# 

# header search path
include_directories (${PROJECT_SOURCE_DIR}/include)

# static library source files
set (src ${PROJECT_SOURCE_DIR}/src)
set (sources ${src}/context.c ${src}/init.c ${src}/input.c ${src}/monitor.c ${src}/vulkan.c ${src}/window.c)
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
    add_definitions (-D_GLFW_WIN32 -D_GLFW_USE_HYBRID_HPG)
    set (sources ${sources} ${src}/win32_init.c ${src}/win32_joystick.c ${src}/win32_monitor.c ${src}/win32_time.c ${src}/win32_tls.c ${src}/win32_window.c ${src}/wgl_context.c ${src}/egl_context.c)
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux)
    add_definitions (-D_GLFW_X11)
    set (sources ${sources} ${src}/x11_init.c ${src}/x11_monitor.c ${src}/x11_window.c ${src}/xkb_unicode.c ${src}/linux_joystick.c ${src}/posix_time.c ${src}/posix_tls.c ${src}/glx_context.c ${src}/egl_context.c)
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
    add_definitions (-D_GLFW_COCOA)
    set (sources ${sources} ${src}/cocoa_init.m ${src}/cocoa_joystick.m ${src}/cocoa_monitor.m ${src}/cocoa_window.m ${src}/cocoa_time.c ${src}/posix_tls.c
 ${src}/nsgl_context.m)

endif ()

# static library
add_library (glfw STATIC ${sources})

# library dependencies and paths
set (GLFW_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
set (GLFW_LIBRARIES glfw ${OPENGL_LIBRARIES})
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
  set (GLFW_LIBRARIES ${GLFW_LIBRARIES} "")
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set (GLFW_LIBRARIES ${GLFW_LIBRARIES} "-framework Cocoa -framework CoreVideo")
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux) 
  set (GLFW_LIBRARIES ${GLFW_LIBRARIES} "X11;Xcursor;Xrandr;Xinerama")
endif ()

# expose to enclosing project
set (GLFW_INCLUDE_DIRS ${GLFW_INCLUDE_DIRS} PARENT_SCOPE)
set (GLFW_LIBRARIES ${GLFW_LIBRARIES} PARENT_SCOPE)


# 
# export package
# 

# export package for use from the build tree
export (TARGETS glfw FILE ${PROJECT_BINARY_DIR}/GLFWTargets.cmake)
export (PACKAGE GLFW)

# package definitions
set (CONF_INCLUDE_DIRS ${GLFW_INCLUDE_DIRS})
set (CONF_LIBRARIES ${GLFW_LIBRARIES})

# package configuration and version files
configure_file (GLFWConfig.cmake.in "${PROJECT_BINARY_DIR}/GLFWConfig.cmake" @ONLY)
configure_file (GLFWConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/GLFWConfigVersion.cmake" @ONLY)
