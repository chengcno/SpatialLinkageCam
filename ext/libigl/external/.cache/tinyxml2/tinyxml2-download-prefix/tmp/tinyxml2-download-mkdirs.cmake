# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/tinyxml2"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake-build-release/tinyxml2-build"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/tmp"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src/tinyxml2-download-stamp"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src/tinyxml2-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src/tinyxml2-download-stamp/${subDir}")
endforeach()
