idf_component_register(SRCS "app_main.cpp" "cam.cpp" "console.cpp" "gps.cpp" "i2chandler.cpp" INCLUDE_DIRS . ulp)

set(COMPONENT_REQUIRES ulp)
