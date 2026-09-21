cmake_minimum_required(VERSION 3.10)
project(pmc VERSION 1.0.0)

set(PMC_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/pmc_heu.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pmc_maxclique.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pmcx_maxclique.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pmcx_maxclique_basic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pmc_cores.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pmc_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pmc_graph.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pmc_clique_utils.cpp
        )

option(PMC_BUILD_SHARED "Build pmc as a shared library (.so)" ON)
if (PMC_BUILD_SHARED)
        add_library(pmc SHARED ${PMC_SOURCE_FILES})
else()
        add_library(pmc STATIC ${PMC_SOURCE_FILES})
        set_property(TARGET pmc PROPERTY POSITION_INDEPENDENT_CODE 1)
endif()

# MSVC's cl rejects GCC/Clang-style -W flags (e.g. /Werror is parsed as a
# numeric warning level and aborts with D8021). Apply these only on
# non-MSVC compilers; MSVC builds use the compiler's defaults.
target_compile_options(pmc PRIVATE
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wextra>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wno-format>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wno-sign-compare>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wno-unused-function>
)

target_include_directories(pmc PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        )

find_package(OpenMP REQUIRED)
target_link_libraries(pmc OpenMP::OpenMP_CXX)

# pmc_main parses CLI flags via getopt() (POSIX-only). Skip the executable
# on Windows; downstream consumers (ROBIN, TEASER++) use only the library.
if(NOT WIN32)
        add_executable(pmc_main pmc_driver.cpp)
        target_link_libraries(pmc_main pmc OpenMP::OpenMP_CXX)
endif()

# Installation
include(GNUInstallDirs)

install(TARGETS pmc
        EXPORT pmc-export
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

export(
        EXPORT pmc-export
        FILE ${CMAKE_CURRENT_BINARY_DIR}/pmcTargets.cmake
        NAMESPACE pmc::
)

install(EXPORT pmc-export
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pmc
        NAMESPACE pmc::
        FILE pmcTargets.cmake
        )

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES cmake/pmcConfig.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pmc
        )
