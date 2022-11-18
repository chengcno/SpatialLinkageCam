# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/cmake/../external/imgui"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/cmake-build-release/imgui-build"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imgui/imgui-download-prefix"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imgui/imgui-download-prefix/tmp"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imgui/imgui-download-prefix/src"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/${subDir}")
endforeach()
