# Micromouse Phase 1 - Simulation Code

This repository contains the simulation code for the Birzeit University **Interfacing Techniques - Micromouse Phase 1** project.


The default algorithm is **Left-Hand Wall Following**. A right-hand mode is also available for optional testing.

> **Windows users:** follow the **Windows Quick Start** below exactly. You do **not** need to build the mms simulator from source.

---

## 1. What is already implemented?

- left-hand wall-following navigation
- optional right-hand wall-following mode
- automatic maze-size detection using `mazeWidth()` and `mazeHeight()`
- automatic center detection for 8x8 and 16x16 mazes
- mouse position and heading tracking
- discovered-wall visualization in mms
- visited-cell visualization and visit counts
- move and turn counters
- unique cells visited
- total cell entries including revisits
- elapsed program time
- `run.log` step-by-step logging
- `results.csv` summary output
- a maximum-step safety stop
- unit tests for wall-following logic, heading updates, movement, center detection, and visit tracking

The main wall-following rule is:

```text
if LEFT is open:
    turn left
    move forward
else if FRONT is open:
    move forward
else if RIGHT is open:
    turn right
    move forward
else:
    turn around
    move forward
```

This is intentionally a basic heuristic. Floodfill/BFS and other team contributions should be added separately.
---

# 2. Windows Quick Start

The steps below are the recommended setup for every teammate so everyone runs the same code in the same way.

## Step 1 - Put the repository in a simple folder

Clone the GitHub repository into a path with no unusual characters. For example:

```text
C:\Micromouse\interfacingTechniques
```

If using Git from Command Prompt:

```bat
git clone YOUR_GITHUB_REPOSITORY_URL C:\Micromouse\interfacingTechniques
cd /d C:\Micromouse\interfacingTechniques
```

Replace `YOUR_GITHUB_REPOSITORY_URL` with the real repository URL.

---

## Step 2 - Download the mms simulator for Windows

Use the official mms project:

https://github.com/mackorone/mms

Download the precompiled Windows release from:

https://github.com/mackorone/mms/releases

Download `windows.zip`, extract it, and run:

```text
mms.exe
```

The official mms documentation says the Windows release is already compiled, so you do not need Qt or Qt Creator just to run the simulator.

Windows SmartScreen may warn about the downloaded application. If Windows blocks the official mms executable, use **More info -> Run anyway** after verifying that it came from the official mackorone/mms release page.

---

## Step 3 - Install a C++ compiler on Windows

This project needs **g++ with C++17 support**.

The recommended setup is **MSYS2 UCRT64 GCC**.

Download and install MSYS2 from:

https://www.msys2.org/

After installation, open the **MSYS2 UCRT64** terminal and run:

```sh
pacman -S mingw-w64-ucrt-x86_64-gcc
```

Then add this folder to the Windows system/user `PATH`:

```text
C:\msys64\ucrt64\bin
```

After changing PATH:

1. close Command Prompt
2. close mms if it is already open
3. open a new Command Prompt
4. check the compiler:

```bat
g++ --version
```

If a GCC version is displayed, the compiler is ready.

If Windows says:

```text
'g++' is not recognized...
```

then the PATH is not configured correctly yet.

---

## Step 4 - Build the program once from Command Prompt

Open **Command Prompt** and go to the repository:

```bat
cd /d C:\Micromouse\interfacingTechniques
```

Build Sama's program:

```bat
build_windows.bat
```

A successful build should end with:

```text
BUILD SUCCESSFUL: wall_follower.exe
```

The generated executable is:

```text
wall_follower.exe
```

Do not upload this `.exe` file to GitHub. It is generated locally.

---

## Step 5 - Run the unit tests

Before opening the simulator, verify the non-simulator logic:

```bat
test_windows.bat
```

A successful test run should print:

```text
All wall-following tests passed.
TESTS PASSED.
```

These tests verify the algorithm decisions, direction changes, position updates, center-cell detection, and visit tracking.

They do **not** replace testing inside mms.

---

# 3. Configure Sama's Code in mms on Windows

Open `mms.exe`.

Click the **+** button in the simulator to add a new algorithm.

Enter the following settings.

| mms field | Value |
|---|---|
| **Name** | `Sama Wall Follower` |
| **Directory** | `C:\Micromouse\interfacingTechniques` |
| **Build Command** | `build_windows.bat` |
| **Run Command** | `wall_follower.exe` |

Change the directory if you cloned the repository somewhere else.

### Important

The **Directory** must be the repository root - the folder that directly contains:

```text
README.md
build_windows.bat
src
tests
```

Do not point mms directly to `src`.

After configuring the algorithm:

1. choose/load the maze you want to test
2. select `Sama Wall Follower`
3. click **Build** if the simulator shows a separate Build button
4. click **Run**

mms communicates with the program through standard input/output. The project therefore reserves **stdout for mms commands** and sends debugging information to **stderr and log files**.

Official mms documentation also uses this stdin/stdout communication model.

---

# 4. Left-Hand vs Right-Hand Mode

## Default: Left-Hand Wall Following

Use this mms Run Command:

```text
wall_follower.exe
```

The priority is:

```text
LEFT -> FRONT -> RIGHT -> TURN AROUND
```

## Optional: Right-Hand Wall Following

Create a second mms algorithm entry if desired.

Use:

| mms field | Value |
|---|---|
| **Name** | `Sama Wall Follower - Right` |
| **Directory** | `C:\Micromouse\interfacingTechniques` |
| **Build Command** | `build_windows.bat` |
| **Run Command** | `wall_follower.exe --right` |

The priority becomes:

```text
RIGHT -> FRONT -> LEFT -> TURN AROUND
```

The assignment work should use the left-hand version as Sama's main baseline unless the team decides otherwise.

---

# 5. Output Files After a Run

The program generates files in the repository root.

## `run.log`

Contains the step-by-step details of the most recent run, for example:

```text
Step: 12
Position: (...)
Heading: ...
Left wall: ...
Front wall: ...
Right wall: ...
Action: ...
Unique cells visited: ...
```

`run.log` is overwritten when a new program instance starts.

## `results.csv`

Contains one summary row for each completed run.

The columns are:

```text
timestamp
algorithm
maze_width
maze_height
goal_reached
moves
turns
unique_cells_visited
total_cell_entries
elapsed_ms
```

`results.csv` is useful for the final comparison table.

Both `run.log` and `results.csv` are intentionally ignored by Git because they are generated experiment output rather than source code.

If a run is important for the report, copy the needed results/screenshots into a separate report/evidence folder before running more experiments.

---

# 7. Project Structure

```text
interfacingTechniques/
|
|-- README.md
|-- .gitignore
|-- Makefile
|-- build_windows.bat
|-- test_windows.bat
|
|-- src/
|   |-- main.cpp
|   |-- API.h
|   |-- API.cpp
|   |
|   |-- simulator/
|   |   |-- IMouseIO.h
|   |   |-- MmsMouseIO.h
|   |   `-- MmsMouseIO.cpp
|   |
|   |-- navigation/
|   |   |-- WallFollowMode.h
|   |   |-- WallFollower.h
|   |   `-- WallFollower.cpp
|   |
|   |-- model/
|   |   |-- Position.h
|   |   |-- Pose.h
|   |   |-- Direction.h
|   |   `-- Direction.cpp
|   |
|   |-- maze/
|   |   |-- GoalDetector.h
|   |   `-- GoalDetector.cpp
|   |
|   |-- tracking/
|   |   |-- RunMetrics.h
|   |   |-- RunMetrics.cpp
|   |   |-- VisitTracker.h
|   |   `-- VisitTracker.cpp
|   |
|   |-- visualization/
|   |   |-- MazeVisualizer.h
|   |   `-- MazeVisualizer.cpp
|   |
|   `-- logging/
|       |-- RunLogger.h
|       `-- RunLogger.cpp
|
`-- tests/
    `-- test_wall_following.cpp
```

---

# 8. What Each Module Does

### `src/main.cpp`

Coordinates a simulator run. It creates the mouse interface, reads maze dimensions, tracks the pose, calls the wall follower, records metrics, handles visualization, and stops at the goal or safety limit.

### `src/API.h` / `src/API.cpp`

Implements the low-level text protocol used to communicate with mms through stdin/stdout.

### `src/simulator/IMouseIO.h`

Defines the small simulator-independent interface needed by the navigation code.

### `src/simulator/MmsMouseIO.*`

Connects the generic mouse interface to the actual mms API.

### `src/navigation/WallFollower.*`

Contains Sama's wall-following decision logic and movement action execution.

### `src/model/*`

Stores the mouse position, heading, and pose helpers.

### `src/maze/GoalDetector.*`

Determines whether the current cell is one of the center cells. It works dynamically from the maze width and height.

### `src/tracking/*`

Tracks moves, turns, timing, total entries, unique visited cells, and per-cell visit counts.

### `src/visualization/*`

Shows discovered walls and visited cells inside mms.

### `src/logging/*`

Writes step information and final metrics to stderr, `run.log`, and `results.csv`.

### `tests/test_wall_following.cpp`

Contains unit tests that can run without mms.

---


# 11. Troubleshooting on Windows

## `g++ is not recognized`

Make sure this folder is in Windows PATH:

```text
C:\msys64\ucrt64\bin
```

Then close and reopen Command Prompt and mms.

Test with:

```bat
g++ --version
```

---

## mms Build fails but `build_windows.bat` works in Command Prompt

mms may have been opened before the compiler was added to PATH.

Close mms completely and open it again.

---

## `wall_follower.exe` does not exist

Run:

```bat
build_windows.bat
```

from the repository root and read the compiler error.

---

## mms appears to freeze or wait forever

mms and the program communicate through stdin/stdout. Arbitrary debug text must not be printed to stdout because it can corrupt the simulator protocol.

The existing project sends debug information to stderr instead.

---

## The mouse revisits cells

That is possible with wall-following and is one of the reasons this algorithm is only the basic benchmark. Repeated visits do not automatically mean the implementation is wrong.

---

## The safety stop occurs

The program uses a maximum step limit based on maze size so a difficult loop cannot keep the test running forever.

The safety limit is not part of the wall-following decision rule.

---

# 12. Linux/macOS Note

The original `Makefile` is kept for teammates who use a Unix-like development environment.

Typical commands are:

```sh
make
make test
```

For the shared team workflow on Windows, use:

```text
build_windows.bat
test_windows.bat
```

instead.

---
## External Tools Used

- mms Micromouse simulator: https://github.com/mackorone/mms
- MSYS2 for Windows GCC: https://www.msys2.org/

The official mms C++ template also notes that Windows users may need MinGW and that simulator communication is performed through stdin/stdout.
