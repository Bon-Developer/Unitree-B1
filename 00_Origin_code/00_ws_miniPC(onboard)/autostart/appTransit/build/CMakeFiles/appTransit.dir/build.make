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
CMAKE_SOURCE_DIR = /home/unitree/Unitree/autostart/appTransit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/unitree/Unitree/autostart/appTransit/build

# Include any dependencies generated for this target.
include CMakeFiles/appTransit.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/appTransit.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/appTransit.dir/flags.make

CMakeFiles/appTransit.dir/AppSDK.cpp.o: CMakeFiles/appTransit.dir/flags.make
CMakeFiles/appTransit.dir/AppSDK.cpp.o: ../AppSDK.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/Unitree/autostart/appTransit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/appTransit.dir/AppSDK.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/appTransit.dir/AppSDK.cpp.o -c /home/unitree/Unitree/autostart/appTransit/AppSDK.cpp

CMakeFiles/appTransit.dir/AppSDK.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/appTransit.dir/AppSDK.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/Unitree/autostart/appTransit/AppSDK.cpp > CMakeFiles/appTransit.dir/AppSDK.cpp.i

CMakeFiles/appTransit.dir/AppSDK.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/appTransit.dir/AppSDK.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/Unitree/autostart/appTransit/AppSDK.cpp -o CMakeFiles/appTransit.dir/AppSDK.cpp.s

CMakeFiles/appTransit.dir/AppSDK.cpp.o.requires:

.PHONY : CMakeFiles/appTransit.dir/AppSDK.cpp.o.requires

CMakeFiles/appTransit.dir/AppSDK.cpp.o.provides: CMakeFiles/appTransit.dir/AppSDK.cpp.o.requires
	$(MAKE) -f CMakeFiles/appTransit.dir/build.make CMakeFiles/appTransit.dir/AppSDK.cpp.o.provides.build
.PHONY : CMakeFiles/appTransit.dir/AppSDK.cpp.o.provides

CMakeFiles/appTransit.dir/AppSDK.cpp.o.provides.build: CMakeFiles/appTransit.dir/AppSDK.cpp.o


# Object files for target appTransit
appTransit_OBJECTS = \
"CMakeFiles/appTransit.dir/AppSDK.cpp.o"

# External object files for target appTransit
appTransit_EXTERNAL_OBJECTS =

appTransit: CMakeFiles/appTransit.dir/AppSDK.cpp.o
appTransit: CMakeFiles/appTransit.dir/build.make
appTransit: CMakeFiles/appTransit.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/unitree/Unitree/autostart/appTransit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable appTransit"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/appTransit.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/appTransit.dir/build: appTransit

.PHONY : CMakeFiles/appTransit.dir/build

CMakeFiles/appTransit.dir/requires: CMakeFiles/appTransit.dir/AppSDK.cpp.o.requires

.PHONY : CMakeFiles/appTransit.dir/requires

CMakeFiles/appTransit.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/appTransit.dir/cmake_clean.cmake
.PHONY : CMakeFiles/appTransit.dir/clean

CMakeFiles/appTransit.dir/depend:
	cd /home/unitree/Unitree/autostart/appTransit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/unitree/Unitree/autostart/appTransit /home/unitree/Unitree/autostart/appTransit /home/unitree/Unitree/autostart/appTransit/build /home/unitree/Unitree/autostart/appTransit/build /home/unitree/Unitree/autostart/appTransit/build/CMakeFiles/appTransit.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/appTransit.dir/depend

