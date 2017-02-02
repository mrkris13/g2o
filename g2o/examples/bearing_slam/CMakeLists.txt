INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(bearing_slam_library ${G2O_LIB_TYPE}
  edge_se2_pointxy.cpp  se2.h                vertex_point_xy.h
  edge_se2.cpp    edge_se2_pointxy.h    vertex_se2.cpp
  edge_se2.h      rand.h                vertex_point_xy.cpp  vertex_se2.h
  parameter_se2_offset.h parameter_se2_offset.cpp
  simulator.h simulator.cpp
  g2o_tutorial_slam2d_api.h
  types_tutorial_slam2d.h types_tutorial_slam2d.cpp
)

SET_TARGET_PROPERTIES(bearing_slam_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}bearing_slam)
target_compile_options(bearing_slam_library PUBLIC -std=c++11)

TARGET_LINK_LIBRARIES(bearing_slam_library core solver_csparse)

ADD_EXECUTABLE(bearing_slam
  bearing_slam.cpp
)

SET_TARGET_PROPERTIES(bearing_slam PROPERTIES OUTPUT_NAME bearing_slam${EXE_POSTFIX})
target_compile_options(bearing_slam PUBLIC -std=c++11)

TARGET_LINK_LIBRARIES(bearing_slam bearing_slam_library)
