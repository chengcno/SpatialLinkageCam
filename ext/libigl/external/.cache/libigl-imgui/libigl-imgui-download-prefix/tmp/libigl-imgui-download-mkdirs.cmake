# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/cmake/../external/libigl-imgui"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/cmake-build-release/libigl-imgui-build"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/tmp"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/${subDir}")
endforeach()
