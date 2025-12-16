# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/lenovo/esp/esp-idf/components/bootloader/subproject"
  "/home/lenovo/Documents/embedded-systems-toolkit/esp32/001_Hello_world/build/bootloader"
  "/home/lenovo/Documents/embedded-systems-toolkit/esp32/001_Hello_world/build/bootloader-prefix"
  "/home/lenovo/Documents/embedded-systems-toolkit/esp32/001_Hello_world/build/bootloader-prefix/tmp"
  "/home/lenovo/Documents/embedded-systems-toolkit/esp32/001_Hello_world/build/bootloader-prefix/src/bootloader-stamp"
  "/home/lenovo/Documents/embedded-systems-toolkit/esp32/001_Hello_world/build/bootloader-prefix/src"
  "/home/lenovo/Documents/embedded-systems-toolkit/esp32/001_Hello_world/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/lenovo/Documents/embedded-systems-toolkit/esp32/001_Hello_world/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/lenovo/Documents/embedded-systems-toolkit/esp32/001_Hello_world/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
