cmake_minimum_required(VERSION 3.10)

project(free5GRAN)

set (CMAKE_CXX_STANDARD 11)

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
        lib/phy/physical_channel/physical_channel.h)

target_link_libraries(free5GRAN_lib asn1c_lib)

add_compile_options(-g)

add_executable(free5GRAN
        src/main.cc
        src/rf/rf.cpp
        src/rf/rf.h
        src/phy/phy.cpp
        src/phy/phy.h )

find_package(Boost)
find_package (Threads)

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
target_link_libraries(free5GRAN asn1c_lib free5GRAN_lib uhd ${Boost_LIBRARIES} boost_system m fftw3 ${CMAKE_THREAD_LIBS_INIT} config++ boost_log boost_thread boost_log_setup)

install(TARGETS free5GRAN DESTINATION /usr/local/bin)
install(DIRECTORY config DESTINATION $ENV{HOME}/.config/free5GRAN)
install(FILES config/search_cell.cfg DESTINATION $ENV{HOME}/.config/free5GRAN/config RENAME free5GRAN.cfg)