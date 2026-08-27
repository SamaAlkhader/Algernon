@echo off
setlocal

where g++ >nul 2>nul
if errorlevel 1 (
    echo ERROR: g++ was not found.
    echo Install MSYS2 UCRT64 GCC and add C:\msys64\ucrt64\bin to the Windows PATH.
    exit /b 1
)

echo Building Sama Wall Follower...

g++ -std=c++17 -Wall -Wextra -pedantic -I.\src ^
    src\main.cpp ^
    src\API.cpp ^
    src\simulator\MmsMouseIO.cpp ^
    src\navigation\WallFollower.cpp ^
    src\model\Direction.cpp ^
    src\tracking\RunMetrics.cpp ^
    src\tracking\VisitTracker.cpp ^
    src\maze\GoalDetector.cpp ^
    src\visualization\MazeVisualizer.cpp ^
    src\logging\RunLogger.cpp ^
    -o wall_follower.exe

if errorlevel 1 (
    echo BUILD FAILED.
    exit /b 1
)

echo BUILD SUCCESSFUL: wall_follower.exe
exit /b 0
