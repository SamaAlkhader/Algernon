#include "API.h"

#include <iostream>
#include <stdexcept>
#include <string>

int API::mazeWidth()
{
    command("mazeWidth");
    return readInt();
}

int API::mazeHeight()
{
    command("mazeHeight");
    return readInt();
}

bool API::wallFront()
{
    command("wallFront");
    return readBool();
}

bool API::wallLeft()
{
    command("wallLeft");
    return readBool();
}

bool API::wallRight()
{
    command("wallRight");
    return readBool();
}

void API::moveForward()
{
    command("moveForward");
    readAck();
}

void API::turnLeft()
{
    command("turnLeft");
    readAck();
}

void API::turnRight()
{
    command("turnRight");
    readAck();
}

void API::setWall(int x, int y, char direction)
{
    command("setWall " + std::to_string(x) + " " + std::to_string(y) + " " + direction);
}

void API::setColor(int x, int y, char color)
{
    command("setColor " + std::to_string(x) + " " + std::to_string(y) + " " + color);
}

void API::setText(int x, int y, const std::string& text)
{
    command("setText " + std::to_string(x) + " " + std::to_string(y) + " " + text);
}

bool API::wasReset()
{
    command("wasReset");
    return readBool();
}

void API::ackReset()
{
    command("ackReset");
}

int API::readInt()
{
    std::string response;
    std::cin >> response;
    return std::stoi(response);
}

bool API::readBool()
{
    std::string response;
    std::cin >> response;
    return response == "true";
}

void API::readAck()
{
    std::string response;
    std::cin >> response;
    if (response == "crash")
    {
        throw std::runtime_error("mms reported a crash");
    }
    if (response != "ack")
    {
        throw std::runtime_error("Expected mms ack, received: " + response);
    }
}

void API::command(const std::string& commandText)
{
    std::cout << commandText << std::endl;
}
