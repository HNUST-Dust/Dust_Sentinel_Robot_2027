# 业务线程装配 — 门禁 TRD_XXX 保持原名（不加 DUST_ 前缀）
#
# 新增一个线程时：
#   1. 在 project/Kconfig 加 config TRD_XXX
#   2. 在下面照这块追加
#
# if(CONFIG_TRD_XXX)
# 	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/xxx/trd_xxx.cpp)
# 	target_include_directories(app PRIVATE
# 		${CMAKE_CURRENT_SOURCE_DIR}
# 		${CMAKE_CURRENT_SOURCE_DIR}/xxx
# 	)
# endif()

if(CONFIG_TRD_CAN_TX)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/can/trd_can_tx.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/can
	)
endif()

if(CONFIG_TRD_IMU)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/trd_imu.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/imu
	)
endif()

if(CONFIG_TRD_REMOTE)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remote/trd_remote.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/remote
	)
endif()

if(CONFIG_TRD_ECAT)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ecat/trd_ecat.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ecat)
endif()

if(CONFIG_TRD_TEST)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/trd_test.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/test
	)
endif()