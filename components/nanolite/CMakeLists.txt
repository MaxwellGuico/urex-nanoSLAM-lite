set(NANOLITE_SOURCES
    nanolite_core.c
    nanolite_icp.c
    nanolite_map.c
    nanolite_optimize.c
    nanolite_scan.c
    nanolite_slam.c
    nanolite_tof.c
)

if(COMMAND idf_component_register)
    idf_component_register(SRCS ${NANOLITE_SOURCES} INCLUDE_DIRS ".")
else()
    cmake_minimum_required(VERSION 3.16)
    project(nanolite C)

    add_library(nanolite STATIC ${NANOLITE_SOURCES})
    target_include_directories(nanolite PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
    target_compile_features(nanolite PUBLIC c_std_11)
    target_compile_options(nanolite PRIVATE -Wall -Wextra -Werror -pedantic)

    if(NOT MSVC)
        target_link_libraries(nanolite PUBLIC m)
    endif()

    option(NANOLITE_BUILD_TESTS "Build NanoSLAM-Lite host tests" ON)
    if(NANOLITE_BUILD_TESTS)
        enable_testing()
        add_executable(nanolite_tests tests/test_nanolite.c)
        target_link_libraries(nanolite_tests PRIVATE nanolite)
        target_compile_options(nanolite_tests PRIVATE -Wall -Wextra -Werror -pedantic)
        if(MSVC)
            target_compile_options(nanolite_tests PRIVATE /UNDEBUG)
        else()
            target_compile_options(nanolite_tests PRIVATE -UNDEBUG)
        endif()
        add_test(NAME nanolite_tests COMMAND nanolite_tests)
    endif()
endif()
