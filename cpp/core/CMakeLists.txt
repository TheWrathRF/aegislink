add_library(aegis_core STATIC
  src/crc32.cpp
  src/kalman.cpp
  src/protocol.cpp
  src/reassembler.cpp
)
target_include_directories(aegis_core PUBLIC include)
# The static library is linked into the Python extension, a shared object on Linux.
set_target_properties(aegis_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
