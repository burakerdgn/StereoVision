# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/nvidia/32B270B0B2707A65/burak/stereo/project-library

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/build

# Include any dependencies generated for this target.
include CMakeFiles/demo_realsense.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/demo_realsense.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demo_realsense.dir/flags.make

CMakeFiles/demo_realsense.dir/realsense.cpp.o: CMakeFiles/demo_realsense.dir/flags.make
CMakeFiles/demo_realsense.dir/realsense.cpp.o: ../realsense.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/nvidia/32B270B0B2707A65/burak/stereo/project-library/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/demo_realsense.dir/realsense.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_realsense.dir/realsense.cpp.o -c /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/realsense.cpp

CMakeFiles/demo_realsense.dir/realsense.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_realsense.dir/realsense.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/realsense.cpp > CMakeFiles/demo_realsense.dir/realsense.cpp.i

CMakeFiles/demo_realsense.dir/realsense.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_realsense.dir/realsense.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/realsense.cpp -o CMakeFiles/demo_realsense.dir/realsense.cpp.s

CMakeFiles/demo_realsense.dir/realsense.cpp.o.requires:

.PHONY : CMakeFiles/demo_realsense.dir/realsense.cpp.o.requires

CMakeFiles/demo_realsense.dir/realsense.cpp.o.provides: CMakeFiles/demo_realsense.dir/realsense.cpp.o.requires
	$(MAKE) -f CMakeFiles/demo_realsense.dir/build.make CMakeFiles/demo_realsense.dir/realsense.cpp.o.provides.build
.PHONY : CMakeFiles/demo_realsense.dir/realsense.cpp.o.provides

CMakeFiles/demo_realsense.dir/realsense.cpp.o.provides.build: CMakeFiles/demo_realsense.dir/realsense.cpp.o


CMakeFiles/demo_realsense.dir/vdisp.cpp.o: CMakeFiles/demo_realsense.dir/flags.make
CMakeFiles/demo_realsense.dir/vdisp.cpp.o: ../vdisp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/nvidia/32B270B0B2707A65/burak/stereo/project-library/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/demo_realsense.dir/vdisp.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_realsense.dir/vdisp.cpp.o -c /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/vdisp.cpp

CMakeFiles/demo_realsense.dir/vdisp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_realsense.dir/vdisp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/vdisp.cpp > CMakeFiles/demo_realsense.dir/vdisp.cpp.i

CMakeFiles/demo_realsense.dir/vdisp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_realsense.dir/vdisp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/vdisp.cpp -o CMakeFiles/demo_realsense.dir/vdisp.cpp.s

CMakeFiles/demo_realsense.dir/vdisp.cpp.o.requires:

.PHONY : CMakeFiles/demo_realsense.dir/vdisp.cpp.o.requires

CMakeFiles/demo_realsense.dir/vdisp.cpp.o.provides: CMakeFiles/demo_realsense.dir/vdisp.cpp.o.requires
	$(MAKE) -f CMakeFiles/demo_realsense.dir/build.make CMakeFiles/demo_realsense.dir/vdisp.cpp.o.provides.build
.PHONY : CMakeFiles/demo_realsense.dir/vdisp.cpp.o.provides

CMakeFiles/demo_realsense.dir/vdisp.cpp.o.provides.build: CMakeFiles/demo_realsense.dir/vdisp.cpp.o


CMakeFiles/demo_realsense.dir/realss.cpp.o: CMakeFiles/demo_realsense.dir/flags.make
CMakeFiles/demo_realsense.dir/realss.cpp.o: ../realss.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/nvidia/32B270B0B2707A65/burak/stereo/project-library/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/demo_realsense.dir/realss.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_realsense.dir/realss.cpp.o -c /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/realss.cpp

CMakeFiles/demo_realsense.dir/realss.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_realsense.dir/realss.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/realss.cpp > CMakeFiles/demo_realsense.dir/realss.cpp.i

CMakeFiles/demo_realsense.dir/realss.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_realsense.dir/realss.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/realss.cpp -o CMakeFiles/demo_realsense.dir/realss.cpp.s

CMakeFiles/demo_realsense.dir/realss.cpp.o.requires:

.PHONY : CMakeFiles/demo_realsense.dir/realss.cpp.o.requires

CMakeFiles/demo_realsense.dir/realss.cpp.o.provides: CMakeFiles/demo_realsense.dir/realss.cpp.o.requires
	$(MAKE) -f CMakeFiles/demo_realsense.dir/build.make CMakeFiles/demo_realsense.dir/realss.cpp.o.provides.build
.PHONY : CMakeFiles/demo_realsense.dir/realss.cpp.o.provides

CMakeFiles/demo_realsense.dir/realss.cpp.o.provides.build: CMakeFiles/demo_realsense.dir/realss.cpp.o


# Object files for target demo_realsense
demo_realsense_OBJECTS = \
"CMakeFiles/demo_realsense.dir/realsense.cpp.o" \
"CMakeFiles/demo_realsense.dir/vdisp.cpp.o" \
"CMakeFiles/demo_realsense.dir/realss.cpp.o"

# External object files for target demo_realsense
demo_realsense_EXTERNAL_OBJECTS =

demo_realsense: CMakeFiles/demo_realsense.dir/realsense.cpp.o
demo_realsense: CMakeFiles/demo_realsense.dir/vdisp.cpp.o
demo_realsense: CMakeFiles/demo_realsense.dir/realss.cpp.o
demo_realsense: CMakeFiles/demo_realsense.dir/build.make
demo_realsense: /opt/ros/melodic/lib/librealsense2.so.2.33.1
demo_realsense: /usr/lib/libopencv_dnn.so.3.3.1
demo_realsense: /usr/lib/libopencv_ml.so.3.3.1
demo_realsense: /usr/lib/libopencv_objdetect.so.3.3.1
demo_realsense: /usr/lib/libopencv_shape.so.3.3.1
demo_realsense: /usr/lib/libopencv_stitching.so.3.3.1
demo_realsense: /usr/lib/libopencv_superres.so.3.3.1
demo_realsense: /usr/lib/libopencv_videostab.so.3.3.1
demo_realsense: /usr/lib/libopencv_calib3d.so.3.3.1
demo_realsense: /usr/lib/libopencv_features2d.so.3.3.1
demo_realsense: /usr/lib/libopencv_flann.so.3.3.1
demo_realsense: /usr/lib/libopencv_highgui.so.3.3.1
demo_realsense: /usr/lib/libopencv_photo.so.3.3.1
demo_realsense: /usr/lib/libopencv_video.so.3.3.1
demo_realsense: /usr/lib/libopencv_videoio.so.3.3.1
demo_realsense: /usr/lib/libopencv_imgcodecs.so.3.3.1
demo_realsense: /usr/lib/libopencv_imgproc.so.3.3.1
demo_realsense: /usr/lib/libopencv_core.so.3.3.1
demo_realsense: CMakeFiles/demo_realsense.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/nvidia/32B270B0B2707A65/burak/stereo/project-library/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable demo_realsense"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo_realsense.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demo_realsense.dir/build: demo_realsense

.PHONY : CMakeFiles/demo_realsense.dir/build

CMakeFiles/demo_realsense.dir/requires: CMakeFiles/demo_realsense.dir/realsense.cpp.o.requires
CMakeFiles/demo_realsense.dir/requires: CMakeFiles/demo_realsense.dir/vdisp.cpp.o.requires
CMakeFiles/demo_realsense.dir/requires: CMakeFiles/demo_realsense.dir/realss.cpp.o.requires

.PHONY : CMakeFiles/demo_realsense.dir/requires

CMakeFiles/demo_realsense.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demo_realsense.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demo_realsense.dir/clean

CMakeFiles/demo_realsense.dir/depend:
	cd /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/nvidia/32B270B0B2707A65/burak/stereo/project-library /media/nvidia/32B270B0B2707A65/burak/stereo/project-library /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/build /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/build /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/build/CMakeFiles/demo_realsense.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/demo_realsense.dir/depend

