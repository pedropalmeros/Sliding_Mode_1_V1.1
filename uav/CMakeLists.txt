PROJECT(Att_Quat_01)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_VISION_FILTER TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

SET(SRC_FILES
	src/main.cpp
	src/PosApp.cpp

	src/xyBackstepping/xyBackstepping.cpp
	src/xyBackstepping/xyBackstepping_impl.cpp

	src/zBackstepping/zBackstepping.cpp
	src/zBackstepping/zBackstepping_impl.cpp

	src/attQuatBkstp/attQuatBkstp.cpp
	src/attQuatBkstp/attQuatBkstp_impl.cpp

	src/xyPD/xyPD.cpp
	src/xyPD/xyPD_impl.cpp

	src/attSMC/attSMC.cpp
	src/attSMC/attSMC_impl.cpp
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR CircleFollower
           COPY_RESOURCES
)
