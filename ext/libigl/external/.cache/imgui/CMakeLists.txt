# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.1)

project(imgui-download NONE)

include(ExternalProject)
ExternalProject_Add(imgui-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/ocornut/imgui.git;GIT_TAG;61b19489f1ba35934d9114c034b24eb5bff149e7
                    SOURCE_DIR          "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/cmake/../external/imgui"
                    BINARY_DIR          "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/cmake-build-release/imgui-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
