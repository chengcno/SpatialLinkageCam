# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if(EXISTS "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/stb/stb-download-prefix/src/stb-download-stamp/stb-download-gitclone-lastrun.txt" AND EXISTS "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/stb/stb-download-prefix/src/stb-download-stamp/stb-download-gitinfo.txt" AND
  "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/stb/stb-download-prefix/src/stb-download-stamp/stb-download-gitclone-lastrun.txt" IS_NEWER_THAN "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/stb/stb-download-prefix/src/stb-download-stamp/stb-download-gitinfo.txt")
  message(STATUS
    "Avoiding repeated git clone, stamp file is up to date: "
    "'/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/stb/stb-download-prefix/src/stb-download-stamp/stb-download-gitclone-lastrun.txt'"
  )
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/stb"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/stb'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/opt/homebrew/bin/git" -c http.sslVerify=false
            clone --no-checkout --config "advice.detachedHead=false" --config "advice.detachedHead=false" "https://github.com/libigl/libigl-stb.git" "stb"
    WORKING_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external"
    RESULT_VARIABLE error_code
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/libigl/libigl-stb.git'")
endif()

execute_process(
  COMMAND "/opt/homebrew/bin/git" -c http.sslVerify=false
          checkout "cd0fa3fcd90325c83be4d697b00214e029f94ca3" --
  WORKING_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/stb"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'cd0fa3fcd90325c83be4d697b00214e029f94ca3'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/opt/homebrew/bin/git" -c http.sslVerify=false
            submodule update --recursive --init 
    WORKING_DIRECTORY "/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/stb"
    RESULT_VARIABLE error_code
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/Users/yingjie/Documents/spatialLink/ext/libigl/cmake/../external/stb'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/stb/stb-download-prefix/src/stb-download-stamp/stb-download-gitinfo.txt" "/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/stb/stb-download-prefix/src/stb-download-stamp/stb-download-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/Users/yingjie/Documents/spatialLink/ext/libigl/external/.cache/stb/stb-download-prefix/src/stb-download-stamp/stb-download-gitclone-lastrun.txt'")
endif()
