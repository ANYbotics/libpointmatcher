find_package(Boost REQUIRED COMPONENTS chrono date_time filesystem program_options system thread timer)

find_package(yaml-cpp QUIET)

if(NOT yaml-cpp_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(yaml-cpp REQUIRED IMPORTED_TARGET yaml-cpp)
  set(yaml_cpp_target "PkgConfig::yaml-cpp")
else()
  set(yaml_cpp_target "yaml-cpp")
endif()

