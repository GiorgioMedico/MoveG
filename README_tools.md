# CppTemplate

This project template is designed to kickstart a new C++ project using CMake and includes various tools to ensure code quality, maintainability, and ease of development.

## Project Overview

This CMake project integrates several powerful tools and configurations to help with:

- **Static Analysis**: Detect potential bugs and code style issues.
- **Dynamic Analysis**: Identify runtime issues like memory leaks.
- **Documentation**: Generate detailed project documentation.
- **Code Formatting**: Automatically format C++ and CMake files.
- **Testing**: Write and run unit tests to ensure code correctness.
- **Continuous Integration**: Use GitHub Actions for automated checks on every push or pull request.

## Tools Integrated

### 1. **Clang-Tidy**
- **Description**: Static analysis and linting tool for C++.
- **Purpose**: Helps catch bugs and enforces coding standards at compile-time.
- **Usage**: Run using the `_clangtidy` build target in Visual Studio Code.

### 2. **Sanitizers**
- **Description**: Runtime analysis tools for C++.
- **Purpose**: Detect memory errors, race conditions, and other runtime issues.
- **Usage**: Enabled by building the project with the `sanitizers` target activated in CMake.

### 3. **Doxygen**
- **Description**: Documentation generator for C++ projects.
- **Purpose**: Generates HTML documentation from source code comments.
- **Usage**: Select the `docs` build target in Visual Studio Code.

### 4. **Clang-Format**
- **Description**: Automatic code formatter for C++ files.
- **Purpose**: Ensures consistent formatting of source files.
- **Usage**: Select the `run_clang_format` build target.

### 5. **CMake-Format**
- **Description**: Formatter for CMakeLists.txt and CMake configuration files.
- **Purpose**: Enforces consistent formatting for CMake files.
- **Usage**: Select the `run_cmake_format` build target.

### 6. **Code Coverage**
- **Description**: Measures the code covered by unit tests.
- **Purpose**: Helps to evaluate how thoroughly the codebase is tested.
- **Usage**: Select the `coverage` build target.

### 7. **Pre-commit Hooks**
- **Description**: Automatically runs formatting and documentation checks before committing.
- **Purpose**: Enforces code standards and documentation at commit time.
- **Usage**: Configured using the `.pre-commit-config.yaml` file.

To set up and run pre-commit hooks, use the following commands in your terminal, in the project directory:

```sh
pre-commit install
pre-commit install-hooks
pre-commit run --all-files
# If any checks fail, fix the issues, then:
git add .
git commit -m "fix pre-commit"
git push
```

### 8. **GitHub Actions**
- **Description**: Continuous Integration (CI) tool on GitHub.
- **Purpose**: Automatically runs static analysis, sanitizers, and documentation generation on every push or pull request.
- **Usage**: Configured via workflows in the `.github/workflows` directory (The repository must be public to use CodeQL).

### 9. **Catch2 and CTest**
- **Description**: Framework for unit testing in C++.
- **Purpose**: Facilitates writing and running unit tests to ensure code reliability.
- **Usage**: Write test cases in the `tests` directory and run using the built-in `Run CTest` command in Visual Studio Code.
