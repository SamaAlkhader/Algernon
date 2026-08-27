#include "logging/RunLogger.h"

#include "model/Direction.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{
std::string yesNo(bool value)
{
    return value ? "YES" : "NO";
}

std::string csvTimestamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
    std::ostringstream out;
    out << std::put_time(std::localtime(&nowTime), "%Y-%m-%d %H:%M:%S");
    return out.str();
}
}

RunLogger::RunLogger(const std::string& logPath)
    : logFile_(logPath, std::ios::out)
{
}

void RunLogger::logDecision(int step,
                            const Pose& poseBeforeMove,
                            const WallReadings& walls,
                            Action action,
                            int uniqueCellsVisited)
{
    std::ostringstream line;
    line << "Step: " << step
         << " | Position: (" << poseBeforeMove.position.x << ", " << poseBeforeMove.position.y << ")"
         << " | Heading: " << directionToString(poseBeforeMove.heading)
         << " | Left wall: " << yesNo(walls.left)
         << " | Front wall: " << yesNo(walls.front)
         << " | Right wall: " << yesNo(walls.right)
         << " | Action: " << actionToString(action)
         << " | Unique cells visited: " << uniqueCellsVisited;
    writeLine(line.str());
}

void RunLogger::logMessage(const std::string& message)
{
    writeLine(message);
}

void RunLogger::writeSummary(const RunMetrics& metrics, int mazeWidth, int mazeHeight)
{
    std::ostringstream summary;
    summary << "========== RUN SUMMARY ==========\n"
            << "Algorithm: " << wallFollowModeToString(metrics.mode()) << "\n"
            << "Maze: " << mazeWidth << " x " << mazeHeight << "\n"
            << "Goal reached: " << yesNo(metrics.goalReached()) << "\n"
            << "Final cell: (" << metrics.finalPosition().x << ", " << metrics.finalPosition().y << ")\n"
            << "Moves: " << metrics.moves() << "\n"
            << "Turns: " << metrics.turns() << "\n"
            << "Unique cells visited: " << metrics.uniqueCellsVisited() << "\n"
            << "Total cell entries: " << metrics.totalCellEntries() << "\n"
            << "Elapsed program time ms: " << metrics.elapsedMilliseconds() << "\n"
            << "=================================";
    writeLine(summary.str());
}

void RunLogger::appendCsvResult(const RunMetrics& metrics, int mazeWidth, int mazeHeight, const std::string& csvPath)
{
    std::ifstream existing(csvPath);
    const bool needsHeader = !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
    existing.close();

    std::ofstream csv(csvPath, std::ios::app);
    if (needsHeader)
    {
        csv << "timestamp,algorithm,maze_width,maze_height,goal_reached,moves,turns,unique_cells_visited,total_cell_entries,elapsed_ms\n";
    }

    csv << csvTimestamp() << ","
        << '"' << wallFollowModeToString(metrics.mode()) << '"' << ","
        << mazeWidth << ","
        << mazeHeight << ","
        << (metrics.goalReached() ? "true" : "false") << ","
        << metrics.moves() << ","
        << metrics.turns() << ","
        << metrics.uniqueCellsVisited() << ","
        << metrics.totalCellEntries() << ","
        << metrics.elapsedMilliseconds() << "\n";
}

void RunLogger::writeLine(const std::string& line)
{
    std::cerr << line << std::endl;
    if (logFile_.is_open())
    {
        logFile_ << line << std::endl;
    }
}
