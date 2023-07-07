#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2016, CHAI3D.
#  (www.chai3d.org)
#
#  Modified by YuanSu(963133358@qq.com) on 2023-June-21



# common settings required to build examples
include_directories (${GLFW_INCLUDE_DIRS})

# build all targets
foreach (example 01-HandExo_Cartesian 02-HandExo_Joint  09-magnets)

  file (GLOB source ${example}/*.cpp)
  add_executable (${example} ${source})
  target_link_libraries (${example} ${CHAI3D_LIBRARIES} ${GLFW_LIBRARIES})
  
  # OS specific adjustments
  if (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
    add_custom_command (TARGET ${example} POST_BUILD COMMAND Rez -append ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../resources/icons/chai3d.rsrc -o ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${example} COMMAND SetFile -a C ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${example} VERBATIM)
  endif ()

endforeach ()
