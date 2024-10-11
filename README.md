# Template For C++ Projects

![C++](https://img.shields.io/badge/C%2B%2B-11%2F14%2F17%2F20%2F23-blue)
[![Ubuntu CI Test](https://github.com/GiorgioMedico/MoveG/actions/workflows/ubuntu.yml/badge.svg?branch=main)](https://github.com/GiorgioMedico/MoveG/actions/workflows/ubuntu.yml)
[![Documentation](https://github.com/GiorgioMedico/MoveG/actions/workflows/documentation.yml/badge.svg?branch=main)](https://github.com/GiorgioMedico/MoveG/actions/workflows/documentation.yml)
[![CodeQL](https://github.com/GiorgioMedico/MoveG/actions/workflows/codeql.yml/badge.svg?branch=main)](https://github.com/GiorgioMedico/MoveG/actions/workflows/codeql.yml)
[![Pre-Commit](https://github.com/GiorgioMedico/MoveG/actions/workflows/pre-commit.yml/badge.svg?branch=main)](https://github.com/GiorgioMedico/MoveG/actions/workflows/pre-commit.yml)

This is a template for modern C++ projects.
What you get is:

- Library, executable and test code separated in distinct folders
- Use of modern CMake for building and compiling
- External libraries installed and managed by
  - [CPM](https://github.com/cpm-cmake/CPM.cmake) Package Manager
- Unit testing using [Catch2](https://github.com/catchorg/Catch2) v2
- General purpose libraries: [spdlog](https://github.com/gabime/spdlog), [cxxopts](https://github.com/jarro2783/cxxopts) and [fmt](https://github.com/fmtlib/fmt)
- Continuous integration testing with Github Actions and [pre-commit](https://pre-commit.com/)
- Code documentation with [Doxygen](https://doxygen.nl/) and [Github Pages](https://franneck94.github.io/CppProjectTemplate/)
- Tooling: Clang-Format, Cmake-Format, Clang-tidy, Sanitizers

## Structure

```text
├── CMakeLists.txt
├── app
│   ├── CMakeLists.txt
│   └── main.cc
├── cmake
│   └── cmake modules
├── docs
│   ├── Doxyfile
│   └── html/
├── src
│   ├── CMakeLists.txt
│   ├── foo/
│   └── bar/
└── tests
  ├── CMakeLists.txt
  └── test_*.cc
```

Library code goes into [src/](src/), main program code in [app/](app) and tests go in [tests/](tests/).

## Software Requirements

- CMake 3.21+
- GNU Makefile
- Doxygen
- G++9 (or higher), Clang++9 (or higher)
- Code Coverage
- Makefile, Doxygen

## Building

First, clone this repo and do the preliminary work:

```shell
git clone --recursive https://github.com/GiorgioMedico/MoveG.git
mkdir build
```

- App Executable, you can use the Visual Studio Code build task or:

```shell
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release --target main
cd app
./main
```

- Unit testing, you can use the Visual Studio Code Run CTest button or:

```shell
cmake -H. -Bbuild -DCMAKE_BUILD_TYPE="Debug"
cmake --build build --config Debug
cd build
ctest .
```

- Documentation, you can use the Visual Studio Code build task 'docs' or:

```shell
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build . --config Debug --target docs
```

- Code Coverage, you can use the Visual Studio Code build task 'coverage' or:

```shell
cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=On
cmake --build build --config Debug --target coverage -j4
cd build
ctest .
```

- To delete cache and reconfigure:

Press `Alt+S` in Visual Studio Code.

For more info about CMake see [here](./README_cmake.md) and [here](./README_tools.md).
