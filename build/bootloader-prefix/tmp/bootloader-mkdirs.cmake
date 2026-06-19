# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/Tep chuong trinh/.espressif/v6.0.1/esp-idf/components/bootloader/subproject")
  file(MAKE_DIRECTORY "D:/Tep chuong trinh/.espressif/v6.0.1/esp-idf/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "D:/Tai_lieu/EE4552/gateway/build/bootloader"
  "D:/Tai_lieu/EE4552/gateway/build/bootloader-prefix"
  "D:/Tai_lieu/EE4552/gateway/build/bootloader-prefix/tmp"
  "D:/Tai_lieu/EE4552/gateway/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Tai_lieu/EE4552/gateway/build/bootloader-prefix/src"
  "D:/Tai_lieu/EE4552/gateway/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Tai_lieu/EE4552/gateway/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Tai_lieu/EE4552/gateway/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
