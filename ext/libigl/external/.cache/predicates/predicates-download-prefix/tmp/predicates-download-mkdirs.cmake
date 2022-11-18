# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/predicates"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake-build-release/predicates-build"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/predicates/predicates-download-prefix"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/predicates/predicates-download-prefix/tmp"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/predicates/predicates-download-prefix/src/predicates-download-stamp"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/predicates/predicates-download-prefix/src"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/predicates/predicates-download-prefix/src/predicates-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/predicates/predicates-download-prefix/src/predicates-download-stamp/${subDir}")
endforeach()
