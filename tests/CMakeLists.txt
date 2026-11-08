find_package(GTest REQUIRED)
include(CheckIncludeFileCXX)
check_include_file_cxx(boost/json.hpp HAVE_BOOST_JSON)

set(TEST_SOURCES
    test_crash_monitor.cc
    test_web_viewer_mirror.cc
    test_atlas_lifecycle.cc
    test_imu_rotation_history.cc
    test_stereo_initialization_points.cc
    test_place_recognition_policy.cc
    test_gyro_relative_accumulator.cc
    test_merge_imu_history.cc
    test_orientation_euler.cc
    test_merge_imu_shadow_policy.cc
)

if(HAVE_BOOST_JSON)
    list(APPEND TEST_SOURCES test_auto_exposure_tuning.cc)
endif()

add_executable(orb_slam3_tests ${TEST_SOURCES})

# The top-level CMakeLists reassigns CMAKE_RUNTIME_OUTPUT_DIRECTORY many times
# for the example targets, so the last value would leak into this subdirectory.
# Pin the test binary next to the build tree instead.
set_target_properties(orb_slam3_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

target_include_directories(orb_slam3_tests PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/CameraModels
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(orb_slam3_tests
    ${PROJECT_NAME}
    GTest::gtest
    GTest::gtest_main
)

add_test(NAME orb_slam3_tests COMMAND orb_slam3_tests)
