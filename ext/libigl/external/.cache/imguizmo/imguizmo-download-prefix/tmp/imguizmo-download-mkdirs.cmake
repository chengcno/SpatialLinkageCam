# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/cmake/../external/imguizmo"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/cmake-build-release/imguizmo-build"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imguizmo/imguizmo-download-prefix"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imguizmo/imguizmo-download-prefix/tmp"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imguizmo/imguizmo-download-prefix/src/imguizmo-download-stamp"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imguizmo/imguizmo-download-prefix/src"
  "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imguizmo/imguizmo-download-prefix/src/imguizmo-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/external/.cache/imguizmo/imguizmo-download-prefix/src/imguizmo-download-stamp/${subDir}")
endforeach()
