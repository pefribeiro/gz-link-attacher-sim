cmake_minimum_required(VERSION 3.16)
project(gz_link_attacher_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(LinkAttacher SHARED src/LinkAttacher.cc)
target_link_libraries(LinkAttacher
  PRIVATE
    gz-sim8::core
    gz-transport13::core
    gz-msgs10::core
    gz-plugin2::register
)

install(TARGETS LinkAttacher DESTINATION lib)
