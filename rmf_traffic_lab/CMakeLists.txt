cmake_minimum_required(VERSION 3.16)

project(rmf_traffic_lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rmf_traffic REQUIRED)

add_executable(rmf_core_lab src/rmf_core_lab.cpp)
target_link_libraries(rmf_core_lab PRIVATE rmf_traffic::rmf_traffic)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rmf_core_lab PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_test(
  NAME bidirectional_corridor_negotiation_runs
  COMMAND rmf_core_lab
    --scenario single_lane_bidirectional
    --output ${CMAKE_CURRENT_BINARY_DIR}/single_lane_bidirectional_test.jsonl)
add_test(
  NAME planner_chooses_short_path
  COMMAND rmf_core_lab
    --scenario single_path
    --output ${CMAKE_CURRENT_BINARY_DIR}/single_path_test.jsonl)
add_test(
  NAME planner_respects_lane_closure
  COMMAND rmf_core_lab
    --scenario single_path_closed
    --output ${CMAKE_CURRENT_BINARY_DIR}/single_path_closed_test.jsonl)
add_test(
  NAME planner_uses_speed_limits
  COMMAND rmf_core_lab
    --scenario speed_limit_choice
    --output ${CMAKE_CURRENT_BINARY_DIR}/speed_limit_choice_test.jsonl)
add_test(
  NAME planner_detects_disconnected_graph
  COMMAND rmf_core_lab
    --scenario disconnected
    --output ${CMAKE_CURRENT_BINARY_DIR}/disconnected_test.jsonl)

install(TARGETS rmf_core_lab RUNTIME DESTINATION bin)
