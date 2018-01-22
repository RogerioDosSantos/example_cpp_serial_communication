
# SERIAL_COMMUNICATION documentation

# Introduction

This example show how to communicate with a serial port.

The current version is designed to work on Windows using only the Windows APIs. 

## How to compile

### Windows

- Open *The project on Visual Studio (2017 or later)* using the *Open CMAKE* option.
- Compile the desired projects

## Output location and details

  The compiled binaries will be located on the */stage* folder which follows the structure below:

      /stage
      ├── build
      │   └── cmake - CMake project files that can be used on to find the libraries by other projects
      │       ├── project-config.cmake
      │       └── project-configversion.cmake
      ├── include
      │       └── project.h
      └── Windows-10.0.15063 - It can change depending on the platform
          └── AMD64 - It can change depending on the platform
              ├── bin
              │   └── project - Executable or shared libraries
              ├── lib
              │   └── project.a - Static libraries
              └── cmake - CMake target for that specific platform
                  ├── project.cmake
                  └── project-noconfig.cmake

