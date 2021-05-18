cmake_minimum_required(VERSION 3.10)

project(free5GRAN)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -std=c++17")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast")

file(GLOB nr_rrc_src
        "lib/asn1c/nr_rrc/*.h"
        "lib/asn1c/nr_rrc/*.c"
        )

ADD_LIBRARY(asn1c_lib STATIC
        ${nr_rrc_src})


ADD_LIBRARY( free5GRAN_lib STATIC
        lib/phy/synchronization/synchronization.cpp
        lib/phy/synchronization/synchronization.h
        lib/utils/sequence_generator/sequence_generator.cpp
        lib/utils/sequence_generator/sequence_generator.h
        lib/utils/common_utils/common_utils.cpp
        lib/utils/common_utils/common_utils.h
        lib/variables/common_structures/common_structures.cpp
        lib/variables/common_structures/common_structures.h
        lib/phy/transport_channel/transport_channel.cpp
        lib/phy/transport_channel/transport_channel.h
        lib/phy/libphy/libphy.cpp
        lib/phy/libphy/libphy.h
        lib/variables/common_variables/common_variables.h
        lib/variables/common_variables/common_variables.cpp
        lib/variables/common_matrices/common_matrices.cpp
        lib/variables/common_matrices/common_matrices.h
        lib/variables/ldpc_matrices/ldpc_matrices.h
        lib/variables/ldpc_matrices/ldpc_matrices.cpp
        lib/asn1c/nr_rrc/asn_internal.h
        lib/asn1c/nr_rrc/asn_application.h
        lib/phy/physical_channel/physical_channel.cpp
        lib/phy/physical_channel/physical_channel.h
        lib/rf/rf.h
        lib/rf/usrp_b200/usrp_b200.cpp
        lib/rf/usrp_b200/usrp_b200.h
        lib/rf/usrp_x300/usrp_x300.cpp
        lib/rf/usrp_x300/usrp_x300.h
        lib/phy/bwp/bwp.cpp
        lib/phy/bwp/bwp.h)

target_link_libraries(free5GRAN_lib asn1c_lib ${GR_LIBRARIES} ${Boost_LIBRARIES} boost_system boost_log boost_thread boost_log_setup)

if (INCLUDE_N210)
    message("INCLUDING N210")
    add_definitions(-DINCLUDE_N210_OPT)
    target_sources(free5GRAN_lib PRIVATE
            lib/rf/usrp_usrp2/usrp_usrp2.cpp
            lib/rf/usrp_usrp2/usrp_usrp2.h)
    target_link_libraries(free5GRAN_lib liquid)
endif()

if (DEBUG_MODE)
    message("DEBUGGING MODE")
    add_compile_options(-g)
endif()

add_executable(free5GRAN
        src/main.cc
        src/phy/phy.cpp
        src/phy/phy.h)

find_package(Boost)
find_package (Threads)



ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
target_link_libraries(free5GRAN asn1c_lib ${GR_LIBRARIES} free5GRAN_lib uhd ${Boost_LIBRARIES} boost_system m fftw3 ${CMAKE_THREAD_LIBS_INIT} config++ boost_log boost_thread boost_log_setup)

install(TARGETS free5GRAN DESTINATION /usr/local/bin)
install(DIRECTORY config DESTINATION /root/.config/free5GRAN)
install(FILES config/search_cell.cfg DESTINATION /root/.config/free5GRAN/config RENAME free5GRAN.cfg)
install(FILES analyse.py DESTINATION /root/.files/free5GRAN/)
install(DIRECTORY DESTINATION /root/.files/free5GRAN/execution_raw_files)
install(DIRECTORY DESTINATION /root/.files/free5GRAN/visualization_files)