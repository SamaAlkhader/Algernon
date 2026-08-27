@echo off
setlocal

where g++ >nul 2>nul
if errorlevel 1 (
    echo ERROR: g++ was not found.
    echo Install MSYS2 UCRT64 GCC and add C:\msys64\ucrt64\bin to the Windows PATH.
    exit /b 1
)

echo Building unit tests...

g++ -std=c++17 -Wall -Wextra -pedantic -I.\src ^
    tests\test_wall_following.cpp ^
    src\navigation\WallFollower.cpp ^
    src\model\Direction.cpp ^
    src\tracking\VisitTracker.cpp ^
    src\maze\GoalDetector.cpp ^
    -o wall_follower_tests.exe

if errorlevel 1 (
    echo TEST BUILD FAILED.
    exit /b 1
)

echo Running unit tests...
wall_follower_tests.exe

if errorlevel 1 (
    echo TESTS FAILED.
    exit /b 1
)

echo TESTS PASSED.
exit /b 0
