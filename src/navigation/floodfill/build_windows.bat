@echo off
REM Builds the C++ Floodfill program for use in mms on Windows.
REM Matches the same MSYS2 UCRT64 g++ toolchain the team already set up.

g++ -std=c++17 -Wall -Wextra -O2 -o floodfill_mouse.exe src\main.cpp src\API.cpp src\FloodfillMicromouse.cpp

if %ERRORLEVEL% EQU 0 (
    echo BUILD SUCCESSFUL: floodfill_mouse.exe
) else (
    echo BUILD FAILED
    exit /b 1
)