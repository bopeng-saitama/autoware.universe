cmake_minimum_required(VERSION 3.16)
project(lidar_feature_localization)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-pg -g -O3 -Wall -Wextra -Wpedantic -Werror)

set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(fmt REQUIRED)

ament_auto_find_build_dependencies()

set(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(LIBRARY_SOURCE_FILES
  src/lib/algorithm.cpp
  src/lib/lib.cpp
  src/lib/eigen.cpp
  src/lib/pcl_utils.cpp
  src/lib/ros_msg.cpp
  src/lib/random.cpp
  src/lib/stats.cpp)

set(LOCALIZATION_SOURCE_FILES
  src/localization/alignment.cpp
  src/localization/degenerate.cpp
  src/localization/edge.cpp
  src/localization/robust.cpp
  src/localization/jacobian.cpp
  src/localization/kdtree.cpp
  src/localization/optimizer.cpp
  src/localization/posevec.cpp)

set(EXTRACTION_SOURCE_FILES
  src/extraction/convolution.cpp
  src/extraction/curvature.cpp
  src/extraction/index_range.cpp
  src/extraction/iterator.cpp
  src/extraction/label.cpp
  src/extraction/math.cpp
  src/extraction/ring.cpp)

ament_auto_add_library(library SHARED ${LIBRARY_SOURCE_FILES})
ament_auto_add_library(localization SHARED ${LOCALIZATION_SOURCE_FILES})
ament_auto_add_library(extraction SHARED ${EXTRACTION_SOURCE_FILES})

target_include_directories(library PUBLIC ${INCLUDE_DIRS})
target_include_directories(localization PUBLIC ${INCLUDE_DIRS})
target_include_directories(extraction PUBLIC ${INCLUDE_DIRS})

target_link_libraries(library ${PCL_LIBRARIES})
target_link_libraries(localization fmt)
target_link_libraries(extraction fmt)

ament_auto_add_executable("lidar_feature_localization" app/localization.cpp)
target_link_libraries("lidar_feature_localization" library localization extraction)

function(add_localization_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name_suffix ${filename})
  set(test_name "localization_${test_name_suffix}")

  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries(${test_name} library localization)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

function(add_extraction_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name_suffix ${filename})
  set(test_name "extraction_${test_name_suffix}")

  message("test_name_suffix = ${test_name_suffix}")
  message("test_name = ${test_name}")
  message("filepath = ${filepath}")

  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries("${test_name}" library extraction)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  file(GLOB
    FILES_TO_CHECK
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${FILES_TO_CHECK})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${FILES_TO_CHECK})

  find_package(ament_cmake_gmock REQUIRED)

  set(LOCALIZATION_TEST_FILES
    test/localization/test_posevec.cpp
    test/localization/test_optimizer.cpp
    test/localization/test_jacobian.cpp
    test/localization/test_edge.cpp
    test/localization/test_recent_scans.cpp
    test/localization/test_robust.cpp
    test/localization/test_kdtree.cpp
    test/localization/test_degenerate.cpp
    test/localization/test_odometry.cpp
    test/localization/test_pointcloud_to_matrix.cpp
    test/localization/test_stamp_sorted_objects.cpp
    test/localization/test_filter.cpp
    test/localization/test_math.cpp
    test/localization/test_alignment.cpp)

  foreach(filepath ${LOCALIZATION_TEST_FILES})
    add_localization_testcase(${filepath})
  endforeach()

  set(EXTRACTION_TEST_FILES
    test/extraction/test_iterator.cpp
    test/extraction/test_algorithm.cpp
    test/extraction/test_convolution.cpp
    test/extraction/test_mapped_points.cpp
    test/extraction/test_curvature.cpp
    test/extraction/test_occlusion.cpp
    test/extraction/test_out_of_range.cpp
    test/extraction/test_range_message.cpp
    test/extraction/test_range.cpp
    test/extraction/test_neighbor.cpp
    test/extraction/test_label.cpp
    test/extraction/test_fill.cpp
    test/extraction/test_parallel_beam.cpp
    test/extraction/test_math.cpp
    test/extraction/test_ring.cpp
    test/extraction/test_index_range.cpp)

  foreach(filepath ${EXTRACTION_TEST_FILES})
    add_extraction_testcase(${filepath})
  endforeach()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
