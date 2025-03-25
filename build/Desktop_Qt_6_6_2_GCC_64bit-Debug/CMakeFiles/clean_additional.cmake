# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/SDNS-GUI_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/SDNS-GUI_autogen.dir/ParseCache.txt"
  "SDNS-GUI_autogen"
  )
endif()
