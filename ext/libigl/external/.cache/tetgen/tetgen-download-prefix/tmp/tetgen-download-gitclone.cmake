# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if(EXISTS "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tetgen/tetgen-download-prefix/src/tetgen-download-stamp/tetgen-download-gitclone-lastrun.txt" AND EXISTS "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tetgen/tetgen-download-prefix/src/tetgen-download-stamp/tetgen-download-gitinfo.txt" AND
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tetgen/tetgen-download-prefix/src/tetgen-download-stamp/tetgen-download-gitclone-lastrun.txt" IS_NEWER_THAN "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tetgen/tetgen-download-prefix/src/tetgen-download-stamp/tetgen-download-gitinfo.txt")
  message(STATUS
    "Avoiding repeated git clone, stamp file is up to date: "
    "'/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tetgen/tetgen-download-prefix/src/tetgen-download-stamp/tetgen-download-gitclone-lastrun.txt'"
  )
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/tetgen"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/tetgen'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/opt/homebrew/bin/git" -c http.sslVerify=false
            clone --no-checkout --config "advice.detachedHead=false" --config "advice.detachedHead=false" "https://github.com/libigl/tetgen.git" "tetgen"
    WORKING_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external"
    RESULT_VARIABLE error_code
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/libigl/tetgen.git'")
endif()

execute_process(
  COMMAND "/opt/homebrew/bin/git" -c http.sslVerify=false
          checkout "4f3bfba3997f20aa1f96cfaff604313a8c2c85b6" --
  WORKING_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/tetgen"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '4f3bfba3997f20aa1f96cfaff604313a8c2c85b6'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/opt/homebrew/bin/git" -c http.sslVerify=false
            submodule update --recursive --init 
    WORKING_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/tetgen"
    RESULT_VARIABLE error_code
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/tetgen'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tetgen/tetgen-download-prefix/src/tetgen-download-stamp/tetgen-download-gitinfo.txt" "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tetgen/tetgen-download-prefix/src/tetgen-download-stamp/tetgen-download-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/tetgen/tetgen-download-prefix/src/tetgen-download-stamp/tetgen-download-gitclone-lastrun.txt'")
endif()
