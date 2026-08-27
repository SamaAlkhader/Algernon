#ifndef LOGGING_RUNLOGGER_H
#define LOGGING_RUNLOGGER_H

#include "model/Pose.h"
#include "navigation/WallFollower.h"
#include "tracking/RunMetrics.h"

#include <fstream>
#include <string>

class RunLogger
{
public:
    explicit RunLogger(const std::string& logPath);

    void logDecision(int step,
                     const Pose& poseBeforeMove,
                     const WallReadings& walls,
                     Action action,
                     int uniqueCellsVisited);

    void logMessage(const std::string& message);
    void writeSummary(const RunMetrics& metrics, int mazeWidth, int mazeHeight);
    void appendCsvResult(const RunMetrics& metrics, int mazeWidth, int mazeHeight, const std::string& csvPath);

private:
    std::ofstream logFile_;

    void writeLine(const std::string& line);
};

#endif
