# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.1)

project(cgal-download NONE)

include(ExternalProject)
ExternalProject_Add(cgal-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/CGAL/cgal.git;GIT_TAG;f7c3c8212b56c0d6dae63787efc99093f4383415
                    SOURCE_DIR          "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/ext/libigl/cmake/../external/cgal"
                    BINARY_DIR          "/Users/yingjie/Documents/GitHub/SpatialLinkageCam/cmake-build-release/cgal-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
