# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.13

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2018\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2018\bin\cmake\win\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\USER\CLionProjects\image_processor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\USER\CLionProjects\image_processor\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/image_processor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/image_processor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/image_processor.dir/flags.make

CMakeFiles/image_processor.dir/src/bitmap.c.obj: CMakeFiles/image_processor.dir/flags.make
CMakeFiles/image_processor.dir/src/bitmap.c.obj: CMakeFiles/image_processor.dir/includes_C.rsp
CMakeFiles/image_processor.dir/src/bitmap.c.obj: ../src/bitmap.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\USER\CLionProjects\image_processor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/image_processor.dir/src/bitmap.c.obj"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\image_processor.dir\src\bitmap.c.obj   -c C:\Users\USER\CLionProjects\image_processor\src\bitmap.c

CMakeFiles/image_processor.dir/src/bitmap.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/image_processor.dir/src/bitmap.c.i"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\USER\CLionProjects\image_processor\src\bitmap.c > CMakeFiles\image_processor.dir\src\bitmap.c.i

CMakeFiles/image_processor.dir/src/bitmap.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/image_processor.dir/src/bitmap.c.s"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\USER\CLionProjects\image_processor\src\bitmap.c -o CMakeFiles\image_processor.dir\src\bitmap.c.s

CMakeFiles/image_processor.dir/src/improc.cpp.obj: CMakeFiles/image_processor.dir/flags.make
CMakeFiles/image_processor.dir/src/improc.cpp.obj: CMakeFiles/image_processor.dir/includes_CXX.rsp
CMakeFiles/image_processor.dir/src/improc.cpp.obj: ../src/improc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\USER\CLionProjects\image_processor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/image_processor.dir/src/improc.cpp.obj"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\image_processor.dir\src\improc.cpp.obj -c C:\Users\USER\CLionProjects\image_processor\src\improc.cpp

CMakeFiles/image_processor.dir/src/improc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_processor.dir/src/improc.cpp.i"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\USER\CLionProjects\image_processor\src\improc.cpp > CMakeFiles\image_processor.dir\src\improc.cpp.i

CMakeFiles/image_processor.dir/src/improc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_processor.dir/src/improc.cpp.s"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\USER\CLionProjects\image_processor\src\improc.cpp -o CMakeFiles\image_processor.dir\src\improc.cpp.s

CMakeFiles/image_processor.dir/main.cpp.obj: CMakeFiles/image_processor.dir/flags.make
CMakeFiles/image_processor.dir/main.cpp.obj: CMakeFiles/image_processor.dir/includes_CXX.rsp
CMakeFiles/image_processor.dir/main.cpp.obj: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\USER\CLionProjects\image_processor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/image_processor.dir/main.cpp.obj"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\image_processor.dir\main.cpp.obj -c C:\Users\USER\CLionProjects\image_processor\main.cpp

CMakeFiles/image_processor.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_processor.dir/main.cpp.i"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\USER\CLionProjects\image_processor\main.cpp > CMakeFiles\image_processor.dir\main.cpp.i

CMakeFiles/image_processor.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_processor.dir/main.cpp.s"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\USER\CLionProjects\image_processor\main.cpp -o CMakeFiles\image_processor.dir\main.cpp.s

# Object files for target image_processor
image_processor_OBJECTS = \
"CMakeFiles/image_processor.dir/src/bitmap.c.obj" \
"CMakeFiles/image_processor.dir/src/improc.cpp.obj" \
"CMakeFiles/image_processor.dir/main.cpp.obj"

# External object files for target image_processor
image_processor_EXTERNAL_OBJECTS =

image_processor.exe: CMakeFiles/image_processor.dir/src/bitmap.c.obj
image_processor.exe: CMakeFiles/image_processor.dir/src/improc.cpp.obj
image_processor.exe: CMakeFiles/image_processor.dir/main.cpp.obj
image_processor.exe: CMakeFiles/image_processor.dir/build.make
image_processor.exe: CMakeFiles/image_processor.dir/linklibs.rsp
image_processor.exe: CMakeFiles/image_processor.dir/objects1.rsp
image_processor.exe: CMakeFiles/image_processor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\USER\CLionProjects\image_processor\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable image_processor.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\image_processor.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/image_processor.dir/build: image_processor.exe

.PHONY : CMakeFiles/image_processor.dir/build

CMakeFiles/image_processor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\image_processor.dir\cmake_clean.cmake
.PHONY : CMakeFiles/image_processor.dir/clean

CMakeFiles/image_processor.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\USER\CLionProjects\image_processor C:\Users\USER\CLionProjects\image_processor C:\Users\USER\CLionProjects\image_processor\cmake-build-debug C:\Users\USER\CLionProjects\image_processor\cmake-build-debug C:\Users\USER\CLionProjects\image_processor\cmake-build-debug\CMakeFiles\image_processor.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/image_processor.dir/depend
