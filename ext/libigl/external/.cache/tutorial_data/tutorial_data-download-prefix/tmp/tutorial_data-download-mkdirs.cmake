# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/../tutorial/data"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake-build-release/tutorial_data-build"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/tmp"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src"
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/${subDir}")
endforeach()
