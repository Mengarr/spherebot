# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rohan/spherebot/src/control_lib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rohan/spherebot/build/control_lib

# Include any dependencies generated for this target.
include CMakeFiles/control_interfaces.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/control_interfaces.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/control_interfaces.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/control_interfaces.dir/flags.make

CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.o: CMakeFiles/control_interfaces.dir/flags.make
CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.o: /home/rohan/spherebot/src/control_lib/src/geodeticConverter.cpp
CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.o: CMakeFiles/control_interfaces.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rohan/spherebot/build/control_lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.o -MF CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.o.d -o CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.o -c /home/rohan/spherebot/src/control_lib/src/geodeticConverter.cpp

CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rohan/spherebot/src/control_lib/src/geodeticConverter.cpp > CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.i

CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rohan/spherebot/src/control_lib/src/geodeticConverter.cpp -o CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.s

CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.o: CMakeFiles/control_interfaces.dir/flags.make
CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.o: /home/rohan/spherebot/src/control_lib/src/stanleyControl.cpp
CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.o: CMakeFiles/control_interfaces.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rohan/spherebot/build/control_lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.o -MF CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.o.d -o CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.o -c /home/rohan/spherebot/src/control_lib/src/stanleyControl.cpp

CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rohan/spherebot/src/control_lib/src/stanleyControl.cpp > CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.i

CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rohan/spherebot/src/control_lib/src/stanleyControl.cpp -o CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.s

CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.o: CMakeFiles/control_interfaces.dir/flags.make
CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.o: /home/rohan/spherebot/src/control_lib/src/auxilary_arduino.cpp
CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.o: CMakeFiles/control_interfaces.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rohan/spherebot/build/control_lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.o -MF CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.o.d -o CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.o -c /home/rohan/spherebot/src/control_lib/src/auxilary_arduino.cpp

CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rohan/spherebot/src/control_lib/src/auxilary_arduino.cpp > CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.i

CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rohan/spherebot/src/control_lib/src/auxilary_arduino.cpp -o CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.s

CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.o: CMakeFiles/control_interfaces.dir/flags.make
CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.o: /home/rohan/spherebot/src/control_lib/src/kinematic_transforms.cpp
CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.o: CMakeFiles/control_interfaces.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rohan/spherebot/build/control_lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.o -MF CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.o.d -o CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.o -c /home/rohan/spherebot/src/control_lib/src/kinematic_transforms.cpp

CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rohan/spherebot/src/control_lib/src/kinematic_transforms.cpp > CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.i

CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rohan/spherebot/src/control_lib/src/kinematic_transforms.cpp -o CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.s

CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.o: CMakeFiles/control_interfaces.dir/flags.make
CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.o: /home/rohan/spherebot/src/control_lib/src/PID_controller.cpp
CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.o: CMakeFiles/control_interfaces.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rohan/spherebot/build/control_lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.o -MF CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.o.d -o CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.o -c /home/rohan/spherebot/src/control_lib/src/PID_controller.cpp

CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rohan/spherebot/src/control_lib/src/PID_controller.cpp > CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.i

CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rohan/spherebot/src/control_lib/src/PID_controller.cpp -o CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.s

CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.o: CMakeFiles/control_interfaces.dir/flags.make
CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.o: /home/rohan/spherebot/src/control_lib/src/LowPassFilter.cpp
CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.o: CMakeFiles/control_interfaces.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rohan/spherebot/build/control_lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.o -MF CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.o.d -o CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.o -c /home/rohan/spherebot/src/control_lib/src/LowPassFilter.cpp

CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rohan/spherebot/src/control_lib/src/LowPassFilter.cpp > CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.i

CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rohan/spherebot/src/control_lib/src/LowPassFilter.cpp -o CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.s

CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.o: CMakeFiles/control_interfaces.dir/flags.make
CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.o: /home/rohan/spherebot/src/control_lib/src/PathDataLoader.cpp
CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.o: CMakeFiles/control_interfaces.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rohan/spherebot/build/control_lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.o -MF CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.o.d -o CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.o -c /home/rohan/spherebot/src/control_lib/src/PathDataLoader.cpp

CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rohan/spherebot/src/control_lib/src/PathDataLoader.cpp > CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.i

CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rohan/spherebot/src/control_lib/src/PathDataLoader.cpp -o CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.s

# Object files for target control_interfaces
control_interfaces_OBJECTS = \
"CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.o" \
"CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.o" \
"CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.o" \
"CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.o" \
"CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.o" \
"CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.o" \
"CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.o"

# External object files for target control_interfaces
control_interfaces_EXTERNAL_OBJECTS =

libcontrol_interfaces.a: CMakeFiles/control_interfaces.dir/src/geodeticConverter.cpp.o
libcontrol_interfaces.a: CMakeFiles/control_interfaces.dir/src/stanleyControl.cpp.o
libcontrol_interfaces.a: CMakeFiles/control_interfaces.dir/src/auxilary_arduino.cpp.o
libcontrol_interfaces.a: CMakeFiles/control_interfaces.dir/src/kinematic_transforms.cpp.o
libcontrol_interfaces.a: CMakeFiles/control_interfaces.dir/src/PID_controller.cpp.o
libcontrol_interfaces.a: CMakeFiles/control_interfaces.dir/src/LowPassFilter.cpp.o
libcontrol_interfaces.a: CMakeFiles/control_interfaces.dir/src/PathDataLoader.cpp.o
libcontrol_interfaces.a: CMakeFiles/control_interfaces.dir/build.make
libcontrol_interfaces.a: CMakeFiles/control_interfaces.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/rohan/spherebot/build/control_lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library libcontrol_interfaces.a"
	$(CMAKE_COMMAND) -P CMakeFiles/control_interfaces.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/control_interfaces.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/control_interfaces.dir/build: libcontrol_interfaces.a
.PHONY : CMakeFiles/control_interfaces.dir/build

CMakeFiles/control_interfaces.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/control_interfaces.dir/cmake_clean.cmake
.PHONY : CMakeFiles/control_interfaces.dir/clean

CMakeFiles/control_interfaces.dir/depend:
	cd /home/rohan/spherebot/build/control_lib && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rohan/spherebot/src/control_lib /home/rohan/spherebot/src/control_lib /home/rohan/spherebot/build/control_lib /home/rohan/spherebot/build/control_lib /home/rohan/spherebot/build/control_lib/CMakeFiles/control_interfaces.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/control_interfaces.dir/depend

