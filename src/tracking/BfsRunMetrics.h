#ifndef TRACKING_BFSRUNMETRICS_H
#define TRACKING_BFSRUNMETRICS_H

#include "model/Position.h"
#include <chrono>
#include <string>
#include <fstream>

// Standalone metrics/log writer for BfsSolver, kept deliberately separate
// from Sama's RunMetrics/RunLogger so nothing here needs to touch
// WallFollowMode or any of her wall-follower-specific types.
//
// It intentionally targets the SAME output shape the team agreed on
// (results.csv columns, run.log style messages) so Rawan's comparison
// table and heatmap pipeline can consume BFS runs exactly like the other
// two algorithms, without caring which class produced them.
class BfsRunMetrics
{
public:
    BfsRunMetrics();

    void startTimer();
    void stopTimer();

    void recordMove();
    void recordTurn();
    void recordTurnAround();

    void setGoalReached(bool reached);
    void setFinalPosition(const Position& position);
    void setVisitCounts(int uniqueCellsVisited, int totalCellEntries);

    bool goalReached() const;
    int moves() const;
    int turns() const;
    int uniqueCellsVisited() const;
    int totalCellEntries() const;
    Position finalPosition() const;
    long long elapsedMilliseconds() const;

private:
    bool goalReached_ = false;
    int moves_ = 0;
    int turns_ = 0;
    int uniqueCellsVisited_ = 0;
    int totalCellEntries_ = 0;
    Position finalPosition_{0, 0};
    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::time_point end_;
};

// Minimal logger, mirroring RunLogger's on-disk contract (run.log style
// messages + a results.csv row with the SAME column order Sama's
// RunLogger::appendCsvResult uses) without depending on her header.
class BfsRunLogger
{
public:
    explicit BfsRunLogger(const std::string& logPath);

    void logMessage(const std::string& message);
    void logStep(int step, const Position& pos, const std::string& actionName,
                 int uniqueCellsVisited);

    // algorithm label is a plain string ("BFS") instead of WallFollowMode,
    // which is the whole point of not reusing RunMetrics/RunLogger here.
    void writeSummary(const BfsRunMetrics& metrics, int mazeWidth, int mazeHeight,
                       const std::string& algorithmLabel = "BFS");
    void appendCsvResult(const BfsRunMetrics& metrics, int mazeWidth, int mazeHeight,
                          const std::string& csvPath,
                          const std::string& algorithmLabel = "BFS");

private:
    std::ofstream logFile_;
    void writeLine(const std::string& line);
};

#endif
