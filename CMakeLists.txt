cmake_minimum_required(VERSION 3.15)

project(rmp-eval VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable warnings as errors
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

set(INCLUDE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(SOURCE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/source")

add_executable(rmp-eval
  "${SOURCE_DIRECTORY}/main.cpp"
  "${SOURCE_DIRECTORY}/quantileestimator.cpp"
  "${SOURCE_DIRECTORY}/reporter.cpp"
  "${SOURCE_DIRECTORY}/ethercatnictest.cpp"
  "${SOURCE_DIRECTORY}/commandlineparser.cpp"
  "${SOURCE_DIRECTORY}/config.cpp"
)
target_include_directories(rmp-eval PRIVATE
  "${INCLUDE_DIRECTORY}"
)

