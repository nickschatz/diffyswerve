# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_SOURCE_DIR = /home/nick/esp/diffyswerve

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nick/esp/diffyswerve

# Include any dependencies generated for this target.
include main/CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include main/CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include main/CMakeFiles/main.dir/flags.make

main/CMakeFiles/main.dir/src/diffyswerve_main.c.obj: main/CMakeFiles/main.dir/flags.make
main/CMakeFiles/main.dir/src/diffyswerve_main.c.obj: main/src/diffyswerve_main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nick/esp/diffyswerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object main/CMakeFiles/main.dir/src/diffyswerve_main.c.obj"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.dir/src/diffyswerve_main.c.obj   -c /home/nick/esp/diffyswerve/main/src/diffyswerve_main.c

main/CMakeFiles/main.dir/src/diffyswerve_main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.dir/src/diffyswerve_main.c.i"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/nick/esp/diffyswerve/main/src/diffyswerve_main.c > CMakeFiles/main.dir/src/diffyswerve_main.c.i

main/CMakeFiles/main.dir/src/diffyswerve_main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.dir/src/diffyswerve_main.c.s"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/nick/esp/diffyswerve/main/src/diffyswerve_main.c -o CMakeFiles/main.dir/src/diffyswerve_main.c.s

main/CMakeFiles/main.dir/src/motor_control.c.obj: main/CMakeFiles/main.dir/flags.make
main/CMakeFiles/main.dir/src/motor_control.c.obj: main/src/motor_control.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nick/esp/diffyswerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object main/CMakeFiles/main.dir/src/motor_control.c.obj"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.dir/src/motor_control.c.obj   -c /home/nick/esp/diffyswerve/main/src/motor_control.c

main/CMakeFiles/main.dir/src/motor_control.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.dir/src/motor_control.c.i"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/nick/esp/diffyswerve/main/src/motor_control.c > CMakeFiles/main.dir/src/motor_control.c.i

main/CMakeFiles/main.dir/src/motor_control.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.dir/src/motor_control.c.s"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/nick/esp/diffyswerve/main/src/motor_control.c -o CMakeFiles/main.dir/src/motor_control.c.s

main/CMakeFiles/main.dir/src/wheelpod_control.c.obj: main/CMakeFiles/main.dir/flags.make
main/CMakeFiles/main.dir/src/wheelpod_control.c.obj: main/src/wheelpod_control.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nick/esp/diffyswerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object main/CMakeFiles/main.dir/src/wheelpod_control.c.obj"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.dir/src/wheelpod_control.c.obj   -c /home/nick/esp/diffyswerve/main/src/wheelpod_control.c

main/CMakeFiles/main.dir/src/wheelpod_control.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.dir/src/wheelpod_control.c.i"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/nick/esp/diffyswerve/main/src/wheelpod_control.c > CMakeFiles/main.dir/src/wheelpod_control.c.i

main/CMakeFiles/main.dir/src/wheelpod_control.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.dir/src/wheelpod_control.c.s"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/nick/esp/diffyswerve/main/src/wheelpod_control.c -o CMakeFiles/main.dir/src/wheelpod_control.c.s

main/CMakeFiles/main.dir/src/encoder_i2c.c.obj: main/CMakeFiles/main.dir/flags.make
main/CMakeFiles/main.dir/src/encoder_i2c.c.obj: main/src/encoder_i2c.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nick/esp/diffyswerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object main/CMakeFiles/main.dir/src/encoder_i2c.c.obj"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/main.dir/src/encoder_i2c.c.obj   -c /home/nick/esp/diffyswerve/main/src/encoder_i2c.c

main/CMakeFiles/main.dir/src/encoder_i2c.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/main.dir/src/encoder_i2c.c.i"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/nick/esp/diffyswerve/main/src/encoder_i2c.c > CMakeFiles/main.dir/src/encoder_i2c.c.i

main/CMakeFiles/main.dir/src/encoder_i2c.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/main.dir/src/encoder_i2c.c.s"
	cd /home/nick/esp/diffyswerve/main && /bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/nick/esp/diffyswerve/main/src/encoder_i2c.c -o CMakeFiles/main.dir/src/encoder_i2c.c.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/diffyswerve_main.c.obj" \
"CMakeFiles/main.dir/src/motor_control.c.obj" \
"CMakeFiles/main.dir/src/wheelpod_control.c.obj" \
"CMakeFiles/main.dir/src/encoder_i2c.c.obj"

# External object files for target main
main_EXTERNAL_OBJECTS =

main/libmain.a: main/CMakeFiles/main.dir/src/diffyswerve_main.c.obj
main/libmain.a: main/CMakeFiles/main.dir/src/motor_control.c.obj
main/libmain.a: main/CMakeFiles/main.dir/src/wheelpod_control.c.obj
main/libmain.a: main/CMakeFiles/main.dir/src/encoder_i2c.c.obj
main/libmain.a: main/CMakeFiles/main.dir/build.make
main/libmain.a: main/CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nick/esp/diffyswerve/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C static library libmain.a"
	cd /home/nick/esp/diffyswerve/main && $(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean_target.cmake
	cd /home/nick/esp/diffyswerve/main && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
main/CMakeFiles/main.dir/build: main/libmain.a

.PHONY : main/CMakeFiles/main.dir/build

main/CMakeFiles/main.dir/clean:
	cd /home/nick/esp/diffyswerve/main && $(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : main/CMakeFiles/main.dir/clean

main/CMakeFiles/main.dir/depend:
	cd /home/nick/esp/diffyswerve && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nick/esp/diffyswerve /home/nick/esp/diffyswerve/main /home/nick/esp/diffyswerve /home/nick/esp/diffyswerve/main /home/nick/esp/diffyswerve/main/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : main/CMakeFiles/main.dir/depend
