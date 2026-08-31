/*
 * Standard Floodfill Micromouse Algorithm:
 *    1. Dynamic Environment Mapping:
 *       Maintains an internal graph representation of discovered spatial boundaries.
 *       Sensor inputs accurately model physical hardware constraints by restricting local wall detection
 *       exclusively to the robot's immediate sensing range (front, left, right).
 *    2. Goal-Oriented Breadth-First Distance Transformation:
 *       Executes a reverse BFS seeded from designated destination cells.
 *       Computes a scalar distance field over all accessible grid coordinates using currently verified wall bounds.
 *    3. Reactive Map Updates and Reflow Triggering:
 *       Evaluates tri-directional sensor data upon entering each cell.
 *       Newly detected obstacles are added to the internal spatial model, immediately invalidating out-of-date
 *       distance metrics and triggering a complete map re-computation (reflow)
 *    4. Gradient-Descent Traversal Policy:
 *       Determines movement vectors by selecting adjacent, non-blocked nodes exhibiting the minimum distance metric.
 *       This ensures continuous deterministic progression down the spatial gradient toward the goal target.
 *    5. Termination Conditions and Exception Handling:
 *       Execution terminates successfully upon reaching target goal coordinates.
 *       If graph traversal exhausts all reachable paths detecting a valid destination topology
 *       (unreachable goal topology), execution safely halts to prevent continuous execution state traps.
 */

#include "FloodfillMicromouse.h"

#include "API.h"

#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

const int FloodfillMicromouse::X_DIRECTION_OFFSET[4] = {0, 1, 0, -1};
const int FloodfillMicromouse::Y_DIRECTION_OFFSET[4] = {1, 0, -1, 0};
const int FloodfillMicromouse::LEFT_DIRECTION[4] = {3, 0, 1, 2};
const int FloodfillMicromouse::RIGHT_DIRECTION[4] = {1, 2, 3, 0};
const int FloodfillMicromouse::OPPOSITE_DIRECTION[4] = {2, 3, 0, 1};

FloodfillMicromouse::FloodfillMicromouse()
    : moveCount_(0), turnCount_(0), totalCellEntries_(0) {
    // Ask mms for the real maze dimensions instead of hardcoding them 
    mazeWidth_ = API::mazeWidth();
    mazeHeight_ = API::mazeHeight();

    discoveredWalls_.assign(mazeWidth_, std::vector<std::set<int>>(mazeHeight_));

    // The four outer edges of the maze are physical boundaries that are
    // always true, in every maze, before the mouse ever moves
    // (pre-load these without needing to sense them)
    for (int x = 0; x < mazeWidth_; ++x) {
        discoveredWalls_[x][mazeHeight_ - 1].insert(0);  // North edge (top row)
        discoveredWalls_[x][0].insert(2);                // South edge (bottom row)
    }
    for (int y = 0; y < mazeHeight_; ++y) {
        discoveredWalls_[mazeWidth_ - 1][y].insert(1);  // East edge (rightmost column)
        discoveredWalls_[0][y].insert(3);               // West edge (leftmost column)
    }

    // mms always starts the mouse at the bottom-left corner (0, 0),
    // facing North (heading = 0).
    x_ = 0;
    y_ = 0;
    heading_ = 0;

    goalCells_ = computeGoalCells();

    // Compute the very first flood map using only the boundary walls
    // above (nothing discovered yet).
    floodValues_ = floodFill();
}

std::vector<std::pair<int, int>> FloodfillMicromouse::computeGoalCells() const {
    // Standard micromouse goal: the 2x2 block of cells at the exact
    // center of the maze (for an even-sized maze like 8x8 or 16x16
    int leftCenterX = (mazeWidth_ - 1) / 2;
    int rightCenterX = mazeWidth_ / 2;
    int bottomCenterY = (mazeHeight_ - 1) / 2;
    int topCenterY = mazeHeight_ / 2;

    std::set<int> xs = {leftCenterX, rightCenterX};
    std::set<int> ys = {bottomCenterY, topCenterY};

    std::vector<std::pair<int, int>> cells;
    for (int gx : xs) {
        for (int gy : ys) {
            cells.push_back({gx, gy});
        }
    }
    return cells;
}

std::vector<std::vector<int>> FloodfillMicromouse::floodFill() const {
    // Floodfill: starting AT the goal
    // cells (distance = 0), spread outward one ring at a time
    // every cell reachable in 1 step from the goal gets value 1,
    //  in 2 steps gets value 2, and so on, stopping at any wall currently known.
    std::vector<std::vector<int>> floodValues(mazeWidth_, std::vector<int>(mazeHeight_, INF));
    std::deque<std::pair<int, int>> cellQueue;

    // Seed the BFS at all four goal cells simultaneously, each with
    // distance 0 (correctly handles reaching any of the four center)
    // cells counting as finishing.
    for (const auto& goal : goalCells_) {
        floodValues[goal.first][goal.second] = 0;
        cellQueue.push_back(goal);
    }

    while (!cellQueue.empty()) {
        auto [currentX, currentY] = cellQueue.front();
        cellQueue.pop_front();

        for (int direction = 0; direction < 4; ++direction) {
            // Skip any direction already known to be blocked.
            if (discoveredWalls_[currentX][currentY].count(direction)) {
                continue;
            }

            int nextX = currentX + X_DIRECTION_OFFSET[direction];
            int nextY = currentY + Y_DIRECTION_OFFSET[direction];

            // Skip anything outside the maze grid.
            if (nextX < 0 || nextX >= mazeWidth_ || nextY < 0 || nextY >= mazeHeight_) {
                continue;
            }

            // Standard BFS relaxation: only update + re-enqueue a cell
            // if we've just found a shorter path to it than before.
            if (floodValues[nextX][nextY] > floodValues[currentX][currentY] + 1) {
                floodValues[nextX][nextY] = floodValues[currentX][currentY] + 1;
                cellQueue.push_back({nextX, nextY});
            }
        }
    }

    return floodValues;
}

bool FloodfillMicromouse::senseAndRecord() {
    // Detects surrounding walls using the front, left, and right
    // sensors, checks only 3 of the 4 sides of the current
    // cell, so step() re-verifies with
    // wallFront() again right before actually moving
    bool newWallFound = false;

    // Query all 3 available sensors, tagged with the absolute direction
    // each one corresponds to (converted from "relative to the mouse's
    // body" using the lookup tables)
    std::vector<std::pair<int, bool>> sensorReadings = {
        {heading_, API::wallFront()},
        {LEFT_DIRECTION[heading_], API::wallLeft()},
        {RIGHT_DIRECTION[heading_], API::wallRight()},
    };

    for (const auto& [direction, isWall] : sensorReadings) {
        if (isWall && !discoveredWalls_[x_][y_].count(direction)) {
            discoveredWalls_[x_][y_].insert(direction);

            int nextX = x_ + X_DIRECTION_OFFSET[direction];
            int nextY = y_ + Y_DIRECTION_OFFSET[direction];

            // A wall has two sides, record it on the neighbor too, so
            // the neighbor's own flood-fill / movement decisions
            // immediately respect it, even before the mouse ever physically visits that cell.
            if (nextX >= 0 && nextX < mazeWidth_ && nextY >= 0 && nextY < mazeHeight_) {
                discoveredWalls_[nextX][nextY].insert(OPPOSITE_DIRECTION[direction]);
            }

            // Draw movement on mms's display 
            API::setWall(x_, y_, directionToLetter(direction));
            newWallFound = true;
        }
    }

    return newWallFound;
}

char FloodfillMicromouse::directionToLetter(int direction) {
    static const char letters[4] = {'n', 'e', 's', 'w'};
    return letters[direction];
}

void FloodfillMicromouse::face(int targetDirection) {
    // Turns the mouse in place to face targetDirection, using the minimum number of turns:
    //   - 1 step clockwise away        -> turnRight() once
    //   - 1 step counter-clockwise away -> turnLeft() once
    //   - exactly opposite (2 away)    -> turnRight() twice
    int requiredTurn = ((targetDirection - heading_) % 4 + 4) % 4;

    if (requiredTurn == 1) {
        API::turnRight();
        turnCount_ += 1;
    } else if (requiredTurn == 3) {
        API::turnLeft();
        turnCount_ += 1;
    } else if (requiredTurn == 2) {
        API::turnRight();
        API::turnRight();
        turnCount_ += 2;
    }
    // requiredTurn == 0 means no turn needed at all.

    heading_ = targetDirection;
}

bool FloodfillMicromouse::step(int targetDirection) {
    // Turns to face targetDirection, RE-CHECKS with a live sensor
    // reading, and only then actually moves forward.
    face(targetDirection);

    if (API::wallFront()) {
        // Assumption was wrong, this is actually a wall. Record
        // it now that we've confirmed it (same bookkeeping as
        // senseAndRecord: mark it locally, mirror it onto the neighbor)
        if (!discoveredWalls_[x_][y_].count(targetDirection)) {
            discoveredWalls_[x_][y_].insert(targetDirection);

            int nextX = x_ + X_DIRECTION_OFFSET[targetDirection];
            int nextY = y_ + Y_DIRECTION_OFFSET[targetDirection];

            if (nextX >= 0 && nextX < mazeWidth_ && nextY >= 0 && nextY < mazeHeight_) {
                discoveredWalls_[nextX][nextY].insert(OPPOSITE_DIRECTION[targetDirection]);
            }

            API::setWall(x_, y_, directionToLetter(targetDirection));
        }

        // This new wall may invalidate the current flood values (a path that looked open may no longer be), so recompute before
        // anyone tries to use them again.
        floodValues_ = floodFill();
        return false;
    }

    std::string moveResult = API::moveForward();
    if (moveResult == "crash") {
        // Per the official mms docs, "crash" is the
        // exact string reported when a move fails: 
        // re-flow and let the caller re-plan,
        floodValues_ = floodFill();
        return false;
    }

    // Move genuinely succeeded, update our internal position to match reality.
    x_ += X_DIRECTION_OFFSET[targetDirection];
    y_ += Y_DIRECTION_OFFSET[targetDirection];
    moveCount_ += 1;
    totalCellEntries_ += 1;
    return true;
}

void FloodfillMicromouse::writeResults(bool goalReached, int uniqueCellsVisited) const {
    // Appends one summary row to results.csv, and overwrites run.log
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - startTime_)
                          .count();

    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);
    std::ostringstream timestamp;
    timestamp << std::put_time(localTime, "%Y-%m-%dT%H:%M:%S");

    // results.csv accumulates one row per run, append, and write the
    // header only if the file doesn't exist yet.
    std::ifstream existsCheck("results.csv");
    bool fileIsNew = !existsCheck.good();
    existsCheck.close();

    std::ofstream csv("results.csv", std::ios::app);
    if (fileIsNew) {
        csv << "timestamp,algorithm,maze_width,maze_height,goal_reached,"
               "moves,turns,unique_cells_visited,total_cell_entries,elapsed_ms\n";
    }
    csv << timestamp.str() << ",floodfill," << mazeWidth_ << "," << mazeHeight_ << ","
        << (goalReached ? "True" : "False") << "," << moveCount_ << "," << turnCount_ << ","
        << uniqueCellsVisited << "," << totalCellEntries_ << "," << elapsedMs << "\n";
    csv.close();

    // run.log is overwritten each run
    std::ofstream log("run.log");
    log << "Floodfill run summary\n";
    log << "======================\n";
    log << "timestamp: " << timestamp.str() << "\n";
    log << "algorithm: floodfill\n";
    log << "maze_width: " << mazeWidth_ << "\n";
    log << "maze_height: " << mazeHeight_ << "\n";
    log << "goal_reached: " << (goalReached ? "True" : "False") << "\n";
    log << "moves: " << moveCount_ << "\n";
    log << "turns: " << turnCount_ << "\n";
    log << "unique_cells_visited: " << uniqueCellsVisited << "\n";
    log << "total_cell_entries: " << totalCellEntries_ << "\n";
    log << "elapsed_ms: " << elapsedMs << "\n";
    log.close();
}

bool FloodfillMicromouse::run() {
    // Main control loop: repeatedly sense, (re-flow if needed), and
    // step toward the lowest flood value, until the goal is reached,
    // the maze is proven unsolvable with current knowledge
    startTime_ = std::chrono::steady_clock::now();

    API::setColor(x_, y_, 'g');  // mark start of the maze
    for (const auto& goal : goalCells_) {
        API::setColor(goal.first, goal.second, 'r');  // mark goal region
    }

    int visitedCells = 0;
    int maxSteps = mazeWidth_ * mazeHeight_ * 20;  // generous safety bound

    for (int step_i = 0; step_i < maxSteps; ++step_i) {
        // check every iteration: if mms ever repositions the
        // mouse, resync our internal model to match reality
        if (API::wasReset()) {
            x_ = 0;
            y_ = 0;
            heading_ = 0;
            API::ackReset();
            continue;
        }

        visitedCells += 1;

        API::setText(x_, y_, std::to_string(floodValues_[x_][y_]));

        bool atGoal = false;
        for (const auto& goal : goalCells_) {
            if (goal.first == x_ && goal.second == y_) {
                atGoal = true;
                break;
            }
        }

        if (atGoal) {
            API::setColor(x_, y_, 'b');
            std::cerr << "GOAL REACHED IN " << visitedCells << " CELL-VISITS\n";
            std::cerr.flush();  
                               
            writeResults(true, visitedCells);
            return true;
        }

        bool newlyDiscovered = senseAndRecord();
        if (newlyDiscovered) {
            // A wall we didn't know about before might invalidate the
            // current flood map, so recompute before deciding where to move next.
            floodValues_ = floodFill();
        }

        // Look at every direction not already known to be blocked, and
        // pick whichever neighbor has the smallest flood value.
        int bestDirection = -1;
        int lowestFloodValue = floodValues_[x_][y_];
        for (int direction = 0; direction < 4; ++direction) {
            if (discoveredWalls_[x_][y_].count(direction)) {
                continue;
            }
            int nextX = x_ + X_DIRECTION_OFFSET[direction];
            int nextY = y_ + Y_DIRECTION_OFFSET[direction];
            if (nextX < 0 || nextX >= mazeWidth_ || nextY < 0 || nextY >= mazeHeight_) {
                continue;
            }
            if (floodValues_[nextX][nextY] < lowestFloodValue) {
                lowestFloodValue = floodValues_[nextX][nextY];
                bestDirection = direction;
            }
        }

        if (bestDirection == -1) {
            // No neighbor is strictly better than where we're standing
            // right now.
            // re-flow with the very latest wall data 
            floodValues_ = floodFill();

            if (floodValues_[x_][y_] >= INF) {
                // Even with a completely fresh recomputation, this cell
                // is disconnected from every goal cell given everything
                // currently known. This is the only condition under
                // which the algorithm gives up
                std::cerr << "NO PATH TO GOAL FOUND WITH CURRENT KNOWLEDGE. STOPPING...";
                std::cerr.flush();
                writeResults(false, visitedCells);
                return false;
            }

            // Otherwise, the re-flow found a legitimate new direction
            // to consider,loop back and re-evaluate bestDirection
            // with the fresh values.
            continue;
        }

        step(bestDirection);
    }

    std::cerr << "STEP LIMIT REACHED....STOPPING\n";
    std::cerr.flush();
    writeResults(false, visitedCells);
    return false;
}
