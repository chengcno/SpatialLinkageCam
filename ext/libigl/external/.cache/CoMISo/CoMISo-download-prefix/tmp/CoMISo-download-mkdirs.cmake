# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/CoMISo"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake-build-release/CoMISo-build"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/CoMISo/CoMISo-download-prefix"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/CoMISo/CoMISo-download-prefix/tmp"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/CoMISo/CoMISo-download-prefix/src/CoMISo-download-stamp"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/CoMISo/CoMISo-download-prefix/src"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/CoMISo/CoMISo-download-prefix/src/CoMISo-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/CoMISo/CoMISo-download-prefix/src/CoMISo-download-stamp/${subDir}")
endforeach()
