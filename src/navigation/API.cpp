#include "API.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace {

std::ofstream& debugLog() {
    static std::ofstream log("mms_debug_log.txt", std::ios::app);
    return log;
}

// Sends one command to mms and BLOCKS until mms's reply line arrives.
// only for commands documented to actually send a reply.

std::string sendCommandAndWait(const std::vector<std::string>& args) {
    std::ostringstream line;
    for (size_t i = 0; i < args.size(); ++i) {
        if (i > 0) line << " ";
        line << args[i];
    }
    std::string lineStr = line.str();

    debugLog() << "SENT: " << lineStr << "\n";
    debugLog().flush();

    std::cout << lineStr << "\n";
    std::cout.flush();

    std::string response;
    std::getline(std::cin, response);

    debugLog() << "RECEIVED: " << response << "\n";
    debugLog().flush();

    return response;
}

// Sends one command to mms and returns immediately, without trying to
// read a reply, only for commands the official mms docs confirm produce no response.
void sendCommandWithoutResponse(const std::vector<std::string>& args) {
    std::ostringstream line;
    for (size_t i = 0; i < args.size(); ++i) {
        if (i > 0) line << " ";
        line << args[i];
    }
    std::string lineStr = line.str();

    debugLog() << "SENT (no response expected per mms documentation): " << lineStr << "\n";
    debugLog().flush();

    std::cout << lineStr << "\n";
    std::cout.flush();
}

}  // namespace

namespace API {

int mazeWidth() {
    return std::stoi(sendCommandAndWait({"mazeWidth"}));
}

int mazeHeight() {
    return std::stoi(sendCommandAndWait({"mazeHeight"}));
}

bool wallFront() {
    return sendCommandAndWait({"wallFront"}) == "true";
}

bool wallRight() {
    return sendCommandAndWait({"wallRight"}) == "true";
}

bool wallLeft() {
    return sendCommandAndWait({"wallLeft"}) == "true";
}

bool wallBack() {
    return sendCommandAndWait({"wallBack"}) == "true";
}

std::string moveForward(int distance) {
    std::vector<std::string> args = {"moveForward"};
    if (distance != 1) {
        args.push_back(std::to_string(distance));
    }
    return sendCommandAndWait(args);
}

std::string turnRight() {
    return sendCommandAndWait({"turnRight"});
}

std::string turnLeft() {
    return sendCommandAndWait({"turnLeft"});
}

bool wasReset() {
    return sendCommandAndWait({"wasReset"}) == "true";
}

std::string ackReset() {
    return sendCommandAndWait({"ackReset"});
}

void setWall(int x, int y, char direction) {
    sendCommandWithoutResponse({"setWall", std::to_string(x), std::to_string(y), std::string(1, direction)});
}

void clearWall(int x, int y, char direction) {
    sendCommandWithoutResponse({"clearWall", std::to_string(x), std::to_string(y), std::string(1, direction)});
}

void setColor(int x, int y, char color) {
    sendCommandWithoutResponse({"setColor", std::to_string(x), std::to_string(y), std::string(1, color)});
}

void clearColor(int x, int y) {
    sendCommandWithoutResponse({"clearColor", std::to_string(x), std::to_string(y)});
}

void clearAllColor() {
    sendCommandWithoutResponse({"clearAllColor"});
}

void setText(int x, int y, const std::string& text) {
    sendCommandWithoutResponse({"setText", std::to_string(x), std::to_string(y), text});
}

void clearText(int x, int y) {
    sendCommandWithoutResponse({"clearText", std::to_string(x), std::to_string(y)});
}

void clearAllText() {
    sendCommandWithoutResponse({"clearAllText"});
}

}  // namespace API
