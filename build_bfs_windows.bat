@echo off
setlocal

where g++ >nul 2>nul
if errorlevel 1 (
    echo ERROR: g++ was not found.
    echo Install MSYS2 UCRT64 GCC and add C:\msys64\ucrt64\bin to the Windows PATH.
    exit /b 1
)

echo Building BFS Solver...

g++ -std=c++17 -O2 -Wall -Wextra -pedantic -I.\src ^
    src\main_bfs.cpp ^
    src\API.cpp ^
    src\simulator\MmsMouseIO.cpp ^
    src\navigation\BfsSolver.cpp ^
    src\model\Direction.cpp ^
    src\tracking\VisitTracker.cpp ^
    src\tracking\BfsRunMetrics.cpp ^
    src\maze\GoalDetector.cpp ^
    -o bfs_solver.exe

if errorlevel 1 (
    echo BUILD FAILED.
    exit /b 1
)

echo BUILD SUCCESSFUL: bfs_solver.exe
exit /b 0
