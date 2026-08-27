#include "simulator/MmsMouseIO.h"

#include "API.h"

int MmsMouseIO::mazeWidth()
{
    return API::mazeWidth();
}

int MmsMouseIO::mazeHeight()
{
    return API::mazeHeight();
}

bool MmsMouseIO::wallFront()
{
    return API::wallFront();
}

bool MmsMouseIO::wallLeft()
{
    return API::wallLeft();
}

bool MmsMouseIO::wallRight()
{
    return API::wallRight();
}

void MmsMouseIO::moveForward()
{
    API::moveForward();
}

void MmsMouseIO::turnLeft()
{
    API::turnLeft();
}

void MmsMouseIO::turnRight()
{
    API::turnRight();
}

bool MmsMouseIO::wasReset()
{
    return API::wasReset();
}

void MmsMouseIO::ackReset()
{
    API::ackReset();
}
