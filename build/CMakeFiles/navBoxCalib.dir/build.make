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
CMAKE_SOURCE_DIR = /mnt/e/project/navboxcali

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/e/project/navboxcali/build

# Include any dependencies generated for this target.
include CMakeFiles/navBoxCalib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/navBoxCalib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/navBoxCalib.dir/flags.make

CMakeFiles/navBoxCalib.dir/src/main.cpp.o: CMakeFiles/navBoxCalib.dir/flags.make
CMakeFiles/navBoxCalib.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/e/project/navboxcali/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/navBoxCalib.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/navBoxCalib.dir/src/main.cpp.o -c /mnt/e/project/navboxcali/src/main.cpp

CMakeFiles/navBoxCalib.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/navBoxCalib.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/project/navboxcali/src/main.cpp > CMakeFiles/navBoxCalib.dir/src/main.cpp.i

CMakeFiles/navBoxCalib.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/navBoxCalib.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/project/navboxcali/src/main.cpp -o CMakeFiles/navBoxCalib.dir/src/main.cpp.s

CMakeFiles/navBoxCalib.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/navBoxCalib.dir/src/main.cpp.o.requires

CMakeFiles/navBoxCalib.dir/src/main.cpp.o.provides: CMakeFiles/navBoxCalib.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/navBoxCalib.dir/build.make CMakeFiles/navBoxCalib.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/navBoxCalib.dir/src/main.cpp.o.provides

CMakeFiles/navBoxCalib.dir/src/main.cpp.o.provides.build: CMakeFiles/navBoxCalib.dir/src/main.cpp.o


CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o: CMakeFiles/navBoxCalib.dir/flags.make
CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o: ../src/matrix/Vector3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/e/project/navboxcali/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o -c /mnt/e/project/navboxcali/src/matrix/Vector3.cpp

CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/project/navboxcali/src/matrix/Vector3.cpp > CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.i

CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/project/navboxcali/src/matrix/Vector3.cpp -o CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.s

CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o.requires:

.PHONY : CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o.requires

CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o.provides: CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o.requires
	$(MAKE) -f CMakeFiles/navBoxCalib.dir/build.make CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o.provides.build
.PHONY : CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o.provides

CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o.provides.build: CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o


CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o: CMakeFiles/navBoxCalib.dir/flags.make
CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o: ../src/matrix/Matrix3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/e/project/navboxcali/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o -c /mnt/e/project/navboxcali/src/matrix/Matrix3.cpp

CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/project/navboxcali/src/matrix/Matrix3.cpp > CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.i

CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/project/navboxcali/src/matrix/Matrix3.cpp -o CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.s

CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o.requires:

.PHONY : CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o.requires

CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o.provides: CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o.requires
	$(MAKE) -f CMakeFiles/navBoxCalib.dir/build.make CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o.provides.build
.PHONY : CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o.provides

CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o.provides.build: CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o


CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o: CMakeFiles/navBoxCalib.dir/flags.make
CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o: ../src/matrix/Quaternion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/e/project/navboxcali/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o -c /mnt/e/project/navboxcali/src/matrix/Quaternion.cpp

CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/project/navboxcali/src/matrix/Quaternion.cpp > CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.i

CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/project/navboxcali/src/matrix/Quaternion.cpp -o CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.s

CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o.requires:

.PHONY : CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o.requires

CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o.provides: CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o.requires
	$(MAKE) -f CMakeFiles/navBoxCalib.dir/build.make CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o.provides.build
.PHONY : CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o.provides

CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o.provides.build: CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o


CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o: CMakeFiles/navBoxCalib.dir/flags.make
CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o: ../src/sins/Earth.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/e/project/navboxcali/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o -c /mnt/e/project/navboxcali/src/sins/Earth.cpp

CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/project/navboxcali/src/sins/Earth.cpp > CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.i

CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/project/navboxcali/src/sins/Earth.cpp -o CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.s

CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o.requires:

.PHONY : CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o.requires

CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o.provides: CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o.requires
	$(MAKE) -f CMakeFiles/navBoxCalib.dir/build.make CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o.provides.build
.PHONY : CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o.provides

CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o.provides.build: CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o


CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o: CMakeFiles/navBoxCalib.dir/flags.make
CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o: ../src/sins/IMU.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/e/project/navboxcali/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o -c /mnt/e/project/navboxcali/src/sins/IMU.cpp

CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/project/navboxcali/src/sins/IMU.cpp > CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.i

CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/project/navboxcali/src/sins/IMU.cpp -o CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.s

CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o.requires:

.PHONY : CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o.requires

CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o.provides: CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o.requires
	$(MAKE) -f CMakeFiles/navBoxCalib.dir/build.make CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o.provides.build
.PHONY : CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o.provides

CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o.provides.build: CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o


# Object files for target navBoxCalib
navBoxCalib_OBJECTS = \
"CMakeFiles/navBoxCalib.dir/src/main.cpp.o" \
"CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o" \
"CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o" \
"CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o" \
"CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o" \
"CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o"

# External object files for target navBoxCalib
navBoxCalib_EXTERNAL_OBJECTS =

navBoxCalib: CMakeFiles/navBoxCalib.dir/src/main.cpp.o
navBoxCalib: CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o
navBoxCalib: CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o
navBoxCalib: CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o
navBoxCalib: CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o
navBoxCalib: CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o
navBoxCalib: CMakeFiles/navBoxCalib.dir/build.make
navBoxCalib: CMakeFiles/navBoxCalib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/e/project/navboxcali/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable navBoxCalib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/navBoxCalib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/navBoxCalib.dir/build: navBoxCalib

.PHONY : CMakeFiles/navBoxCalib.dir/build

CMakeFiles/navBoxCalib.dir/requires: CMakeFiles/navBoxCalib.dir/src/main.cpp.o.requires
CMakeFiles/navBoxCalib.dir/requires: CMakeFiles/navBoxCalib.dir/src/matrix/Vector3.cpp.o.requires
CMakeFiles/navBoxCalib.dir/requires: CMakeFiles/navBoxCalib.dir/src/matrix/Matrix3.cpp.o.requires
CMakeFiles/navBoxCalib.dir/requires: CMakeFiles/navBoxCalib.dir/src/matrix/Quaternion.cpp.o.requires
CMakeFiles/navBoxCalib.dir/requires: CMakeFiles/navBoxCalib.dir/src/sins/Earth.cpp.o.requires
CMakeFiles/navBoxCalib.dir/requires: CMakeFiles/navBoxCalib.dir/src/sins/IMU.cpp.o.requires

.PHONY : CMakeFiles/navBoxCalib.dir/requires

CMakeFiles/navBoxCalib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/navBoxCalib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/navBoxCalib.dir/clean

CMakeFiles/navBoxCalib.dir/depend:
	cd /mnt/e/project/navboxcali/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/e/project/navboxcali /mnt/e/project/navboxcali /mnt/e/project/navboxcali/build /mnt/e/project/navboxcali/build /mnt/e/project/navboxcali/build/CMakeFiles/navBoxCalib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/navBoxCalib.dir/depend

