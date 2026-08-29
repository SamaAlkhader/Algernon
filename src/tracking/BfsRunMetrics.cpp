#include "tracking/BfsRunMetrics.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

// ---------------------------------------------------------------------
// BfsRunMetrics
// ---------------------------------------------------------------------
BfsRunMetrics::BfsRunMetrics() = default;

void BfsRunMetrics::startTimer() { start_ = std::chrono::steady_clock::now(); }
void BfsRunMetrics::stopTimer()  { end_ = std::chrono::steady_clock::now(); }

void BfsRunMetrics::recordMove() { moves_++; }
void BfsRunMetrics::recordTurn() { turns_++; }
void BfsRunMetrics::recordTurnAround() { turns_ += 2; } // matches a 180 = two quarter turns, consistent with WallFollower's convention

void BfsRunMetrics::setGoalReached(bool reached) { goalReached_ = reached; }
void BfsRunMetrics::setFinalPosition(const Position& position) { finalPosition_ = position; }
void BfsRunMetrics::setVisitCounts(int uniqueCellsVisited, int totalCellEntries)
{
    uniqueCellsVisited_ = uniqueCellsVisited;
    totalCellEntries_ = totalCellEntries;
}

bool BfsRunMetrics::goalReached() const { return goalReached_; }
int BfsRunMetrics::moves() const { return moves_; }
int BfsRunMetrics::turns() const { return turns_; }
int BfsRunMetrics::uniqueCellsVisited() const { return uniqueCellsVisited_; }
int BfsRunMetrics::totalCellEntries() const { return totalCellEntries_; }
Position BfsRunMetrics::finalPosition() const { return finalPosition_; }

long long BfsRunMetrics::elapsedMilliseconds() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_).count();
}

// ---------------------------------------------------------------------
// BfsRunLogger
// ---------------------------------------------------------------------
namespace
{
std::string currentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}
}

BfsRunLogger::BfsRunLogger(const std::string& logPath)
    : logFile_(logPath, std::ios::out | std::ios::trunc)
{
}

void BfsRunLogger::writeLine(const std::string& line)
{
    if (logFile_.is_open())
    {
        logFile_ << line << "\n";
        logFile_.flush();
    }
}

void BfsRunLogger::logMessage(const std::string& message)
{
    writeLine(message);
}

void BfsRunLogger::logStep(int step, const Position& pos, const std::string& actionName,
                            int uniqueCellsVisited)
{
    std::ostringstream oss;
    oss << "Step: " << step
        << " | Position: (" << pos.x << "," << pos.y << ")"
        << " | Action: " << actionName
        << " | Unique cells visited: " << uniqueCellsVisited;
    writeLine(oss.str());
}

void BfsRunLogger::writeSummary(const BfsRunMetrics& metrics, int mazeWidth, int mazeHeight,
                                 const std::string& algorithmLabel)
{
    std::ostringstream oss;
    oss << "\n--- SUMMARY ---\n"
        << "Algorithm: " << algorithmLabel << "\n"
        << "Maze size: " << mazeWidth << " x " << mazeHeight << "\n"
        << "Goal reached: " << (metrics.goalReached() ? "yes" : "no") << "\n"
        << "Moves: " << metrics.moves() << "\n"
        << "Turns: " << metrics.turns() << "\n"
        << "Unique cells visited: " << metrics.uniqueCellsVisited() << "\n"
        << "Total cell entries: " << metrics.totalCellEntries() << "\n"
        << "Final position: (" << metrics.finalPosition().x << "," << metrics.finalPosition().y << ")\n"
        << "Elapsed ms: " << metrics.elapsedMilliseconds();
    writeLine(oss.str());
}

// Column order matches the README's documented results.csv contract exactly:
// timestamp,algorithm,maze_width,maze_height,goal_reached,moves,turns,
// unique_cells_visited,total_cell_entries,elapsed_ms
void BfsRunLogger::appendCsvResult(const BfsRunMetrics& metrics, int mazeWidth, int mazeHeight,
                                    const std::string& csvPath,
                                    const std::string& algorithmLabel)
{
    bool fileExisted = false;
    {
        std::ifstream check(csvPath);
        fileExisted = check.good();
    }

    std::ofstream csv(csvPath, std::ios::out | std::ios::app);
    if (!csv.is_open()) return;

    if (!fileExisted)
    {
        csv << "timestamp,algorithm,maze_width,maze_height,goal_reached,"
               "moves,turns,unique_cells_visited,total_cell_entries,elapsed_ms\n";
    }

    csv << currentTimestamp() << ","
        << algorithmLabel << ","
        << mazeWidth << ","
        << mazeHeight << ","
        << (metrics.goalReached() ? "true" : "false") << ","
        << metrics.moves() << ","
        << metrics.turns() << ","
        << metrics.uniqueCellsVisited() << ","
        << metrics.totalCellEntries() << ","
        << metrics.elapsedMilliseconds() << "\n";
}
