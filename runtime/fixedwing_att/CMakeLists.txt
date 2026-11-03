set(AVANTI_ATT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
add_library(avanti2_att SHARED
    generated/PX4Controller_ertmodel0827.cpp
    generated/PX4Controller_ertmodel0827_data.cpp
    generated/rtGetInf.cpp generated/rtGetNaN.cpp generated/rt_nonfinite.cpp
    wrapper/avanti_att_c_api.cpp)
avanti_configure_runtime(avanti2_att AVANTI_ATT_BUILDING_DLL "${AVANTI_ATT_ROOT}")
find_package(Threads REQUIRED)
target_link_libraries(avanti2_att PRIVATE Threads::Threads)
install(FILES include/avanti_att_c_api.h DESTINATION include)
