# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "scurve_construct_plot/CMakeFiles/scurve_construct_plot_autogen.dir/AutogenUsed.txt"
  "scurve_construct_plot/CMakeFiles/scurve_construct_plot_autogen.dir/ParseCache.txt"
  "scurve_construct_plot/scurve_construct_plot_autogen"
  )
endif()
