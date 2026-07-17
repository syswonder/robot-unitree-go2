cmake_minimum_required(VERSION 3.16)
project(go2_chassis_offline_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GO2_CHASSIS_BUILD_OFFLINE_TESTS "Build tests that never contact a robot" ON)

if(GO2_CHASSIS_BUILD_OFFLINE_TESTS)
  enable_testing()
  add_executable(protocol_guard_test tests/protocol_guard_test.cpp)
  target_include_directories(protocol_guard_test PRIVATE include)
  target_compile_options(protocol_guard_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME protocol_guard_test COMMAND protocol_guard_test)

  add_executable(runtime_graph_test tests/runtime_graph_test.cpp)
  target_include_directories(runtime_graph_test PRIVATE include)
  target_compile_options(runtime_graph_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME runtime_graph_test COMMAND runtime_graph_test)
endif()
