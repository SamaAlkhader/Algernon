@echo off
setlocal

where g++ >nul 2>nul
if errorlevel 1 (
    echo ERROR: g++ was not found.
    exit /b 1
)

g++ -std=c++17 -Wall -Wextra -pedantic -I.\src ^
    tests\test_bfs.cpp ^
    src\navigation\BfsSolver.cpp ^
    src\model\Direction.cpp ^
    src\maze\GoalDetector.cpp ^
    -o bfs_tests.exe

if errorlevel 1 exit /b 1
bfs_tests.exe
if errorlevel 1 exit /b 1

echo BFS TESTS PASSED.
exit /b 0
