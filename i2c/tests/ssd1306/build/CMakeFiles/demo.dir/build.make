# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/developer/sambashare/suniv_drivers/i2c/ssd1306

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/developer/sambashare/suniv_drivers/i2c/ssd1306/build

# Include any dependencies generated for this target.
include CMakeFiles/demo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/demo.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demo.dir/flags.make

CMakeFiles/demo.dir/main.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/main.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/main.c
CMakeFiles/demo.dir/main.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/demo.dir/main.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/main.c.o -MF CMakeFiles/demo.dir/main.c.o.d -o CMakeFiles/demo.dir/main.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/main.c

CMakeFiles/demo.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/main.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/main.c > CMakeFiles/demo.dir/main.c.i

CMakeFiles/demo.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/main.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/main.c -o CMakeFiles/demo.dir/main.c.s

CMakeFiles/demo.dir/src/fonts/font_10x18.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_10x18.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_10x18.c
CMakeFiles/demo.dir/src/fonts/font_10x18.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/demo.dir/src/fonts/font_10x18.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_10x18.c.o -MF CMakeFiles/demo.dir/src/fonts/font_10x18.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_10x18.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_10x18.c

CMakeFiles/demo.dir/src/fonts/font_10x18.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_10x18.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_10x18.c > CMakeFiles/demo.dir/src/fonts/font_10x18.c.i

CMakeFiles/demo.dir/src/fonts/font_10x18.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_10x18.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_10x18.c -o CMakeFiles/demo.dir/src/fonts/font_10x18.c.s

CMakeFiles/demo.dir/src/fonts/font_6x10.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_6x10.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_6x10.c
CMakeFiles/demo.dir/src/fonts/font_6x10.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/demo.dir/src/fonts/font_6x10.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_6x10.c.o -MF CMakeFiles/demo.dir/src/fonts/font_6x10.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_6x10.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_6x10.c

CMakeFiles/demo.dir/src/fonts/font_6x10.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_6x10.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_6x10.c > CMakeFiles/demo.dir/src/fonts/font_6x10.c.i

CMakeFiles/demo.dir/src/fonts/font_6x10.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_6x10.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_6x10.c -o CMakeFiles/demo.dir/src/fonts/font_6x10.c.s

CMakeFiles/demo.dir/src/fonts/font_6x11.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_6x11.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_6x11.c
CMakeFiles/demo.dir/src/fonts/font_6x11.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/demo.dir/src/fonts/font_6x11.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_6x11.c.o -MF CMakeFiles/demo.dir/src/fonts/font_6x11.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_6x11.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_6x11.c

CMakeFiles/demo.dir/src/fonts/font_6x11.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_6x11.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_6x11.c > CMakeFiles/demo.dir/src/fonts/font_6x11.c.i

CMakeFiles/demo.dir/src/fonts/font_6x11.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_6x11.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_6x11.c -o CMakeFiles/demo.dir/src/fonts/font_6x11.c.s

CMakeFiles/demo.dir/src/fonts/font_7x14.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_7x14.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_7x14.c
CMakeFiles/demo.dir/src/fonts/font_7x14.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/demo.dir/src/fonts/font_7x14.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_7x14.c.o -MF CMakeFiles/demo.dir/src/fonts/font_7x14.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_7x14.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_7x14.c

CMakeFiles/demo.dir/src/fonts/font_7x14.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_7x14.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_7x14.c > CMakeFiles/demo.dir/src/fonts/font_7x14.c.i

CMakeFiles/demo.dir/src/fonts/font_7x14.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_7x14.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_7x14.c -o CMakeFiles/demo.dir/src/fonts/font_7x14.c.s

CMakeFiles/demo.dir/src/fonts/font_8x16.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_8x16.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_8x16.c
CMakeFiles/demo.dir/src/fonts/font_8x16.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/demo.dir/src/fonts/font_8x16.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_8x16.c.o -MF CMakeFiles/demo.dir/src/fonts/font_8x16.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_8x16.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_8x16.c

CMakeFiles/demo.dir/src/fonts/font_8x16.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_8x16.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_8x16.c > CMakeFiles/demo.dir/src/fonts/font_8x16.c.i

CMakeFiles/demo.dir/src/fonts/font_8x16.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_8x16.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_8x16.c -o CMakeFiles/demo.dir/src/fonts/font_8x16.c.s

CMakeFiles/demo.dir/src/fonts/font_8x8.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_8x8.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_8x8.c
CMakeFiles/demo.dir/src/fonts/font_8x8.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/demo.dir/src/fonts/font_8x8.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_8x8.c.o -MF CMakeFiles/demo.dir/src/fonts/font_8x8.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_8x8.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_8x8.c

CMakeFiles/demo.dir/src/fonts/font_8x8.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_8x8.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_8x8.c > CMakeFiles/demo.dir/src/fonts/font_8x8.c.i

CMakeFiles/demo.dir/src/fonts/font_8x8.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_8x8.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_8x8.c -o CMakeFiles/demo.dir/src/fonts/font_8x8.c.s

CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_acorn_8x8.c
CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.o -MF CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_acorn_8x8.c

CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_acorn_8x8.c > CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.i

CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_acorn_8x8.c -o CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.s

CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_mini_4x6.c
CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.o -MF CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_mini_4x6.c

CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_mini_4x6.c > CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.i

CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_mini_4x6.c -o CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.s

CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_pearl_8x8.c
CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.o -MF CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_pearl_8x8.c

CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_pearl_8x8.c > CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.i

CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_pearl_8x8.c -o CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.s

CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_sun12x22.c
CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.o -MF CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_sun12x22.c

CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_sun12x22.c > CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.i

CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_sun12x22.c -o CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.s

CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_sun8x16.c
CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.o -MF CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.o.d -o CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_sun8x16.c

CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_sun8x16.c > CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.i

CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/font_sun8x16.c -o CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.s

CMakeFiles/demo.dir/src/fonts/fonts.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/fonts/fonts.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/fonts.c
CMakeFiles/demo.dir/src/fonts/fonts.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building C object CMakeFiles/demo.dir/src/fonts/fonts.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/fonts/fonts.c.o -MF CMakeFiles/demo.dir/src/fonts/fonts.c.o.d -o CMakeFiles/demo.dir/src/fonts/fonts.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/fonts.c

CMakeFiles/demo.dir/src/fonts/fonts.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/fonts/fonts.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/fonts.c > CMakeFiles/demo.dir/src/fonts/fonts.c.i

CMakeFiles/demo.dir/src/fonts/fonts.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/fonts/fonts.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/fonts/fonts.c -o CMakeFiles/demo.dir/src/fonts/fonts.c.s

CMakeFiles/demo.dir/src/ssd1306.c.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/src/ssd1306.c.o: /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/ssd1306.c
CMakeFiles/demo.dir/src/ssd1306.c.o: CMakeFiles/demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building C object CMakeFiles/demo.dir/src/ssd1306.c.o"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/demo.dir/src/ssd1306.c.o -MF CMakeFiles/demo.dir/src/ssd1306.c.o.d -o CMakeFiles/demo.dir/src/ssd1306.c.o -c /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/ssd1306.c

CMakeFiles/demo.dir/src/ssd1306.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/demo.dir/src/ssd1306.c.i"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/ssd1306.c > CMakeFiles/demo.dir/src/ssd1306.c.i

CMakeFiles/demo.dir/src/ssd1306.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/demo.dir/src/ssd1306.c.s"
	/usr/bin/arm-linux-gnueabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/developer/sambashare/suniv_drivers/i2c/ssd1306/src/ssd1306.c -o CMakeFiles/demo.dir/src/ssd1306.c.s

# Object files for target demo
demo_OBJECTS = \
"CMakeFiles/demo.dir/main.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_10x18.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_6x10.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_6x11.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_7x14.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_8x16.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_8x8.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.o" \
"CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.o" \
"CMakeFiles/demo.dir/src/fonts/fonts.c.o" \
"CMakeFiles/demo.dir/src/ssd1306.c.o"

# External object files for target demo
demo_EXTERNAL_OBJECTS =

demo: CMakeFiles/demo.dir/main.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_10x18.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_6x10.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_6x11.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_7x14.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_8x16.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_8x8.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_acorn_8x8.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_mini_4x6.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_pearl_8x8.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_sun12x22.c.o
demo: CMakeFiles/demo.dir/src/fonts/font_sun8x16.c.o
demo: CMakeFiles/demo.dir/src/fonts/fonts.c.o
demo: CMakeFiles/demo.dir/src/ssd1306.c.o
demo: CMakeFiles/demo.dir/build.make
demo: CMakeFiles/demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking C executable demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demo.dir/build: demo
.PHONY : CMakeFiles/demo.dir/build

CMakeFiles/demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demo.dir/clean

CMakeFiles/demo.dir/depend:
	cd /home/developer/sambashare/suniv_drivers/i2c/ssd1306/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/developer/sambashare/suniv_drivers/i2c/ssd1306 /home/developer/sambashare/suniv_drivers/i2c/ssd1306 /home/developer/sambashare/suniv_drivers/i2c/ssd1306/build /home/developer/sambashare/suniv_drivers/i2c/ssd1306/build /home/developer/sambashare/suniv_drivers/i2c/ssd1306/build/CMakeFiles/demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/demo.dir/depend

