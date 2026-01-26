
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_DEPENDENCIES
  libnabo
  Eigen3
  )

find_package(ament_cmake REQUIRED)

foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()

include(cmake/configExtras.cmake)

########################
## Library definition ##
########################

add_library(pointmatcher
  pointmatcher/Logger.cpp
  pointmatcher/Exceptions.cpp
  pointmatcher/DataPoints.cpp
  pointmatcher/Matches.cpp
  pointmatcher/ICP.cpp
  pointmatcher/Registry.cpp
  pointmatcher/Registrar.cpp
  pointmatcher/DataPointsFilter.cpp
  pointmatcher/Matcher.cpp
  pointmatcher/OutlierFilter.cpp
  pointmatcher/ErrorMinimizer.cpp
  pointmatcher/Transformation.cpp
  pointmatcher/TransformationChecker.cpp
  pointmatcher/Inspector.cpp
  pointmatcher/IO.cpp
  pointmatcher/IOFunctions.cpp
  pointmatcher/Bibliography.cpp
  pointmatcher/Timer.cpp
  pointmatcher/Histogram.cpp
  pointmatcher/Parametrizable.cpp
  pointmatcher/LoggerImpl.cpp
  pointmatcher/MatchersImpl.cpp
  pointmatcher/OutlierFiltersImpl.cpp
  pointmatcher/SurfaceNormalEstimatorPCA.cpp
  pointmatcher/TransformationsImpl.cpp
  pointmatcher/TransformationCheckersImpl.cpp
  pointmatcher/InspectorsImpl.cpp
  #ErrorMinimizers
  pointmatcher/ErrorMinimizers/PointToPlane.cpp
  pointmatcher/ErrorMinimizers/Identity.cpp
  #DataPointsFilters	
  pointmatcher/DataPointsFilters/Identity.cpp
  pointmatcher/DataPointsFilters/RemoveNaN.cpp
  pointmatcher/DataPointsFilters/MaxDist.cpp
  pointmatcher/DataPointsFilters/MinDist.cpp
  pointmatcher/DataPointsFilters/BoundingBox.cpp
  pointmatcher/DataPointsFilters/MaxDensity.cpp
  pointmatcher/DataPointsFilters/SurfaceNormal.cpp
  pointmatcher/DataPointsFilters/SamplingSurfaceNormal.cpp
  pointmatcher/DataPointsFilters/OrientNormals.cpp
  pointmatcher/DataPointsFilters/IncidenceAngle.cpp
  pointmatcher/DataPointsFilters/RandomSampling.cpp
  pointmatcher/DataPointsFilters/MaxPointCount.cpp
  pointmatcher/DataPointsFilters/Shadow.cpp
  pointmatcher/DataPointsFilters/ObservationDirection.cpp
  pointmatcher/DataPointsFilters/OctreeGrid.cpp
  pointmatcher/DataPointsFilters/NormalSpace.cpp
  pointmatcher/DataPointsFilters/CovarianceSampling.cpp
  pointmatcher/DataPointsFilters/DistanceLimit.cpp
  pointmatcher/DataPointsFilters/OrganizedCloudSurfaceNormal.cpp
  #PointCloudGenerators
  pointmatcher/PointCloudGenerator.cpp
  )

target_include_directories(pointmatcher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/pointmatcher>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/pointmatcher/DataPointsFilters>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/pointmatcher/DataPointsFilters/utils>
    $<INSTALL_INTERFACE:include/>
    $<INSTALL_INTERFACE:include/pointmatcher>
    $<INSTALL_INTERFACE:include/pointmatcher/DataPointsFilters>
    $<INSTALL_INTERFACE:include/pointmatcher/DataPointsFilters/utils>
    )

target_link_libraries(pointmatcher
  Eigen3::Eigen
  Boost::chrono
  Boost::date_time
  Boost::filesystem
  Boost::program_options
  Boost::thread
  Boost::timer
  Boost::system
  ${yaml_cpp_target}
  )

# NOTE(apoghosov): it seems that at the time of porting to ROS2 we have the following setup in our build pipeline:
# Release - -O3 by default
# RelWithDebInfo - -O2 by default
# but I want to follow what is written in the catkin build variant ... -O3 everywhere except debug. Note that
# specifiying it here will overide the previous option ... last specified is applied
target_compile_options(pointmatcher PRIVATE
  "$<$<NOT:$<CONFIG:Debug>>:-O3>"
  )

set_target_properties(pointmatcher
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
  )

ament_target_dependencies(pointmatcher ${PACKAGE_DEPENDENCIES})

#############
## Install ##
#############

install(DIRECTORY pointmatcher/
  DESTINATION include/pointmatcher
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "testing" EXCLUDE
  )

install(TARGETS pointmatcher
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

#############
## Testing ##
#############
  
if(BUILD_TESTING)
  
  # testing library ...
  add_library(pointmatcher_testing
    pointmatcher/testing/utils_filesystem.cpp
    pointmatcher/testing/utils_filesystem.h
    pointmatcher/testing/utils_geometry.cpp
    pointmatcher/testing/utils_geometry.h
    pointmatcher/testing/utils_gtest.cpp
    pointmatcher/testing/utils_registration.cpp
    pointmatcher/testing/utils_transformations.cpp
    pointmatcher/testing/RegistrationTestCase.cpp
    pointmatcher/testing/RegistrationTestResult.cpp
    pointmatcher/testing/TransformationError.cpp
    )

  target_include_directories(pointmatcher_testing PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/pointmatcher/testing>
    )

  target_link_libraries(pointmatcher_testing
    pointmatcher
    Eigen3::Eigen
    Boost::chrono
    Boost::date_time
    Boost::filesystem
    Boost::program_options
    Boost::thread
    Boost::timer
    Boost::system
    ${yaml_cpp_target}
    )
  
  find_package(ament_cmake_gtest REQUIRED)

  set(SAVE_TEST_DATA_TO_DISK 0)
  set(TEST_DATA_FOLDER "${CMAKE_SOURCE_DIR}/examples/data/")

  ament_add_gtest(test_pointmatcher
    utest/utest.cpp
    utest/ui/DataFilters.cpp
    utest/ui/DataPoints.cpp
    utest/ui/ErrorMinimizers.cpp
    utest/ui/Inspectors.cpp
    utest/ui/IO.cpp
    utest/ui/Loggers.cpp
    utest/ui/Matcher.cpp
    utest/ui/Outliers.cpp
    utest/ui/PointCloudGenerator.cpp
    utest/ui/Transformations.cpp
    utest/ui/octree/Octree.cpp
    utest/ui/icp/GeneralTests.cpp
    utest/ui/icp/Conditioning.cpp
    )

  target_compile_definitions(test_pointmatcher PRIVATE
    SAVE_TEST_DATA_TO_DISK=${SAVE_TEST_DATA_TO_DISK}
	UTEST_TEST_DATA_PATH="${TEST_DATA_FOLDER}/"
    )

  target_link_libraries(test_pointmatcher
    pointmatcher_testing
    )

  ##################
  # Code_coverage ##
  ##################

  find_package(cmake_code_coverage QUIET)

  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
	  TEST_BUILD_TARGETS
        test_pointmatcher
      SOURCE_EXCLUDE_PATTERN
        ${PROJECT_SOURCE_DIR}/utest/*
        ${PROJECT_SOURCE_DIR}/utest/ui/*
      )
  endif()
	
endif()

ament_package(CONFIG_EXTRAS cmake/configExtras.cmake)
