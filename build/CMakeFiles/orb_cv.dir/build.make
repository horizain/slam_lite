# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/horizain/slam_lite

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/horizain/slam_lite/build

# Include any dependencies generated for this target.
include CMakeFiles/orb_cv.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/orb_cv.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/orb_cv.dir/flags.make

CMakeFiles/orb_cv.dir/src/orb_self.cpp.o: CMakeFiles/orb_cv.dir/flags.make
CMakeFiles/orb_cv.dir/src/orb_self.cpp.o: ../src/orb_self.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/horizain/slam_lite/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/orb_cv.dir/src/orb_self.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/orb_cv.dir/src/orb_self.cpp.o -c /home/horizain/slam_lite/src/orb_self.cpp

CMakeFiles/orb_cv.dir/src/orb_self.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/orb_cv.dir/src/orb_self.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/horizain/slam_lite/src/orb_self.cpp > CMakeFiles/orb_cv.dir/src/orb_self.cpp.i

CMakeFiles/orb_cv.dir/src/orb_self.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/orb_cv.dir/src/orb_self.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/horizain/slam_lite/src/orb_self.cpp -o CMakeFiles/orb_cv.dir/src/orb_self.cpp.s

CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.o: CMakeFiles/orb_cv.dir/flags.make
CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.o: ../src/FASTDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/horizain/slam_lite/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.o -c /home/horizain/slam_lite/src/FASTDetector.cpp

CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/horizain/slam_lite/src/FASTDetector.cpp > CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.i

CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/horizain/slam_lite/src/FASTDetector.cpp -o CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.s

# Object files for target orb_cv
orb_cv_OBJECTS = \
"CMakeFiles/orb_cv.dir/src/orb_self.cpp.o" \
"CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.o"

# External object files for target orb_cv
orb_cv_EXTERNAL_OBJECTS =

orb_cv: CMakeFiles/orb_cv.dir/src/orb_self.cpp.o
orb_cv: CMakeFiles/orb_cv.dir/src/FASTDetector.cpp.o
orb_cv: CMakeFiles/orb_cv.dir/build.make
orb_cv: /opt/opencv3416/lib/libopencv_stitching.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_superres.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_videostab.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_aruco.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_bgsegm.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_bioinspired.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_ccalib.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_dnn_objdetect.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_dpm.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_face.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_freetype.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_fuzzy.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_hdf.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_hfs.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_img_hash.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_line_descriptor.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_optflow.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_reg.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_rgbd.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_saliency.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_stereo.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_structured_light.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_surface_matching.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_tracking.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_xfeatures2d.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_ximgproc.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_xobjdetect.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_xphoto.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_shape.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_highgui.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_videoio.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_viz.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_phase_unwrapping.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_video.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_datasets.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_plot.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_text.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_dnn.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_ml.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_imgcodecs.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_objdetect.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_calib3d.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_features2d.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_flann.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_photo.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_imgproc.so.3.4.16
orb_cv: /opt/opencv3416/lib/libopencv_core.so.3.4.16
orb_cv: CMakeFiles/orb_cv.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/horizain/slam_lite/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable orb_cv"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/orb_cv.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/orb_cv.dir/build: orb_cv

.PHONY : CMakeFiles/orb_cv.dir/build

CMakeFiles/orb_cv.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/orb_cv.dir/cmake_clean.cmake
.PHONY : CMakeFiles/orb_cv.dir/clean

CMakeFiles/orb_cv.dir/depend:
	cd /home/horizain/slam_lite/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/horizain/slam_lite /home/horizain/slam_lite /home/horizain/slam_lite/build /home/horizain/slam_lite/build /home/horizain/slam_lite/build/CMakeFiles/orb_cv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/orb_cv.dir/depend

