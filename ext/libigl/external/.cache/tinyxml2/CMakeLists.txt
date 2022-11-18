# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.1)

project(tinyxml2-download NONE)

include(ExternalProject)
ExternalProject_Add(tinyxml2-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/leethomason/tinyxml2.git;GIT_TAG;d175e9de0be0d4db75d0a8cf065599a435a87eb6
                    SOURCE_DIR          "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/tinyxml2"
                    BINARY_DIR          "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake-build-release/tinyxml2-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
