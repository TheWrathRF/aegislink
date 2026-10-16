add_library(aegis_proxy STATIC
  control.cpp
  gilbert_elliott.cpp
  link.cpp
  token_bucket.cpp
  udp_socket.cpp
)
target_include_directories(aegis_proxy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(aegis_proxy PUBLIC aegis_core)
if(WIN32)
  target_link_libraries(aegis_proxy PUBLIC ws2_32)
  target_compile_definitions(aegis_proxy PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

add_executable(ew_chaos_proxy main.cpp)
target_link_libraries(ew_chaos_proxy PRIVATE aegis_proxy)
if(WIN32)
  target_link_libraries(ew_chaos_proxy PRIVATE winmm)
  target_compile_definitions(ew_chaos_proxy PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

if(SKBUILD)
  # pip puts it next to the Python scripts, so it is on PATH in an activated environment.
  install(TARGETS ew_chaos_proxy DESTINATION "${SKBUILD_SCRIPTS_DIR}")
endif()
