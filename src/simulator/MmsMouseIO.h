#ifndef SIMULATOR_MMSMOUSEIO_H
#define SIMULATOR_MMSMOUSEIO_H

#include "simulator/IMouseIO.h"

class MmsMouseIO : public IMouseIO
{
public:
    int mazeWidth() override;
    int mazeHeight() override;

    bool wallFront() override;
    bool wallLeft() override;
    bool wallRight() override;

    void moveForward() override;
    void turnLeft() override;
    void turnRight() override;

    bool wasReset();
    void ackReset();
};

#endif
