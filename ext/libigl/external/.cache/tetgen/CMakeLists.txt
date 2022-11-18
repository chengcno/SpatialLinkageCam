# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.1)

project(tetgen-download NONE)

include(ExternalProject)
ExternalProject_Add(tetgen-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/tetgen.git;GIT_TAG;4f3bfba3997f20aa1f96cfaff604313a8c2c85b6
                    SOURCE_DIR          "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/tetgen"
                    BINARY_DIR          "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake-build-release/tetgen-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
