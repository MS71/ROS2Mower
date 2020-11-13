idf_component_register(SRCS "app_main.cpp" "ros2node.cpp" "cam.cpp" "console.cpp" "gps.cpp" "i2chandler.cpp" "webui.cpp" INCLUDE_DIRS . ulp)
set(COMPONENT_REQUIRES ulp server websocket micro-ROS)
