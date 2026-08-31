#ifndef API_H
#define API_H

#include <string>

namespace API {

int mazeWidth();
int mazeHeight();

bool wallFront();
bool wallRight();
bool wallLeft();
bool wallBack();

// Returns "ack" on success, "crash" on failure (per official mms docs)

std::string moveForward(int distance = 1);

std::string turnRight();
std::string turnLeft();

bool wasReset();
std::string ackReset();


void setWall(int x, int y, char direction);
void clearWall(int x, int y, char direction);

void setColor(int x, int y, char color);
void clearColor(int x, int y);
void clearAllColor();

void setText(int x, int y, const std::string& text);
void clearText(int x, int y);
void clearAllText();

}  

#endif  
