## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# use default install config
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/embree-config-install.cmake)

# and override path variables to match for build directory
SET(EMBREE_INCLUDE_DIRS /Users/yingjie/Documents/spatialLink/ext/libigl/external/embree/include)
SET(EMBREE_LIBRARY /Users/yingjie/Documents/spatialLink/ext/libigl/cmake-build-release/embree/libembree3.a)
SET(EMBREE_LIBRARIES ${EMBREE_LIBRARY})
