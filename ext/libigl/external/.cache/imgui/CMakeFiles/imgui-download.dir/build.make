# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui

# Utility rule file for imgui-download.

# Include the progress variables for this target.
include CMakeFiles/imgui-download.dir/progress.make

CMakeFiles/imgui-download: CMakeFiles/imgui-download-complete


CMakeFiles/imgui-download-complete: imgui-download-prefix/src/imgui-download-stamp/imgui-download-install
CMakeFiles/imgui-download-complete: imgui-download-prefix/src/imgui-download-stamp/imgui-download-mkdir
CMakeFiles/imgui-download-complete: imgui-download-prefix/src/imgui-download-stamp/imgui-download-download
CMakeFiles/imgui-download-complete: imgui-download-prefix/src/imgui-download-stamp/imgui-download-update
CMakeFiles/imgui-download-complete: imgui-download-prefix/src/imgui-download-stamp/imgui-download-patch
CMakeFiles/imgui-download-complete: imgui-download-prefix/src/imgui-download-stamp/imgui-download-configure
CMakeFiles/imgui-download-complete: imgui-download-prefix/src/imgui-download-stamp/imgui-download-build
CMakeFiles/imgui-download-complete: imgui-download-prefix/src/imgui-download-stamp/imgui-download-install
CMakeFiles/imgui-download-complete: imgui-download-prefix/src/imgui-download-stamp/imgui-download-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'imgui-download'"
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E make_directory /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E touch /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles/imgui-download-complete
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E touch /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-done

imgui-download-prefix/src/imgui-download-stamp/imgui-download-install: imgui-download-prefix/src/imgui-download-stamp/imgui-download-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No install step for 'imgui-download'"
	cd /Users/temp/Documents/GitHub/3DFrameWork/cmake-build-release/imgui-build && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E echo_append
	cd /Users/temp/Documents/GitHub/3DFrameWork/cmake-build-release/imgui-build && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E touch /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-install

imgui-download-prefix/src/imgui-download-stamp/imgui-download-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'imgui-download'"
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E make_directory /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/cmake/../external/imgui
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E make_directory /Users/temp/Documents/GitHub/3DFrameWork/cmake-build-release/imgui-build
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E make_directory /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E make_directory /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/tmp
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E make_directory /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E make_directory /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E make_directory /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E touch /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-mkdir

imgui-download-prefix/src/imgui-download-stamp/imgui-download-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitinfo.txt
imgui-download-prefix/src/imgui-download-stamp/imgui-download-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'imgui-download'"
	cd /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -P /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/tmp/imgui-download-gitclone.cmake
	cd /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E touch /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-download

imgui-download-prefix/src/imgui-download-stamp/imgui-download-update: imgui-download-prefix/src/imgui-download-stamp/imgui-download-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing update step for 'imgui-download'"
	cd /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/imgui && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -P /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/tmp/imgui-download-gitupdate.cmake

imgui-download-prefix/src/imgui-download-stamp/imgui-download-patch: imgui-download-prefix/src/imgui-download-stamp/imgui-download-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'imgui-download'"
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E echo_append
	/Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E touch /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-patch

imgui-download-prefix/src/imgui-download-stamp/imgui-download-configure: imgui-download-prefix/tmp/imgui-download-cfgcmd.txt
imgui-download-prefix/src/imgui-download-stamp/imgui-download-configure: imgui-download-prefix/src/imgui-download-stamp/imgui-download-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No configure step for 'imgui-download'"
	cd /Users/temp/Documents/GitHub/3DFrameWork/cmake-build-release/imgui-build && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E echo_append
	cd /Users/temp/Documents/GitHub/3DFrameWork/cmake-build-release/imgui-build && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E touch /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-configure

imgui-download-prefix/src/imgui-download-stamp/imgui-download-build: imgui-download-prefix/src/imgui-download-stamp/imgui-download-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No build step for 'imgui-download'"
	cd /Users/temp/Documents/GitHub/3DFrameWork/cmake-build-release/imgui-build && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E echo_append
	cd /Users/temp/Documents/GitHub/3DFrameWork/cmake-build-release/imgui-build && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E touch /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-build

imgui-download-prefix/src/imgui-download-stamp/imgui-download-test: imgui-download-prefix/src/imgui-download-stamp/imgui-download-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "No test step for 'imgui-download'"
	cd /Users/temp/Documents/GitHub/3DFrameWork/cmake-build-release/imgui-build && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E echo_append
	cd /Users/temp/Documents/GitHub/3DFrameWork/cmake-build-release/imgui-build && /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E touch /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-test

imgui-download: CMakeFiles/imgui-download
imgui-download: CMakeFiles/imgui-download-complete
imgui-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-build
imgui-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-configure
imgui-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-download
imgui-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-install
imgui-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-mkdir
imgui-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-patch
imgui-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-test
imgui-download: imgui-download-prefix/src/imgui-download-stamp/imgui-download-update
imgui-download: CMakeFiles/imgui-download.dir/build.make

.PHONY : imgui-download

# Rule to build all files generated by this target.
CMakeFiles/imgui-download.dir/build: imgui-download

.PHONY : CMakeFiles/imgui-download.dir/build

CMakeFiles/imgui-download.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imgui-download.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imgui-download.dir/clean

CMakeFiles/imgui-download.dir/depend:
	cd /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui /Users/temp/Documents/GitHub/3DFrameWork/ext/libigl/external/.cache/imgui/CMakeFiles/imgui-download.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imgui-download.dir/depend

