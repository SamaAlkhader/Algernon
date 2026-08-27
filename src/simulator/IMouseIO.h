#ifndef SIMULATOR_IMOUSEIO_H
#define SIMULATOR_IMOUSEIO_H

class IMouseIO
{
public:
    virtual ~IMouseIO() = default;

    virtual int mazeWidth() = 0;
    virtual int mazeHeight() = 0;

    virtual bool wallFront() = 0;
    virtual bool wallLeft() = 0;
    virtual bool wallRight() = 0;

    virtual void moveForward() = 0;
    virtual void turnLeft() = 0;
    virtual void turnRight() = 0;
};

#endif
