INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(bearing_slam_library ${G2O_LIB_TYPE}
  rand.h
  simulator.h simulator.cpp
  g2o_tutorial_slam2d_api.h
)

SET_TARGET_PROPERTIES(bearing_slam_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}bearing_slam)
target_compile_options(bearing_slam_library PUBLIC -std=c++11)

TARGET_LINK_LIBRARIES(bearing_slam_library core solver_csparse types_slam2d)

ADD_EXECUTABLE(bearing_slam
  bearing_slam.cpp
)

SET_TARGET_PROPERTIES(bearing_slam PROPERTIES OUTPUT_NAME bearing_slam${EXE_POSTFIX})
target_compile_options(bearing_slam PUBLIC -std=c++11)

TARGET_LINK_LIBRARIES(bearing_slam bearing_slam_library)
