add_library(aegis_test_main STATIC test_main.cpp)
target_include_directories(aegis_test_main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

foreach(name crc32 protocol reassembler fuzz)
  add_executable(test_${name} test_${name}.cpp)
  target_link_libraries(test_${name} PRIVATE aegis_core aegis_test_main)
  add_test(NAME ${name} COMMAND test_${name})
endforeach()
