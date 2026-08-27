#ifndef API_H
#define API_H

#include <string>

class API
{
public:
    static int mazeWidth();
    static int mazeHeight();

    static bool wallFront();
    static bool wallLeft();
    static bool wallRight();

    static void moveForward();
    static void turnLeft();
    static void turnRight();

    static void setWall(int x, int y, char direction);
    static void setColor(int x, int y, char color);
    static void setText(int x, int y, const std::string& text);

    static bool wasReset();
    static void ackReset();

private:
    static int readInt();
    static bool readBool();
    static void readAck();
    static void command(const std::string& command);
};

#endif
