
#ifndef OBSTACLE_H
#define OBSTACLE_H

using namespace std;

class OBSTACLE
{
public:
    //For Init
    OBSTACLE()
    {

    }
    OBSTACLE(int x, int y)
    {
        this->x = x;
        this->y = y;
    }

    float x = 0.0;
    float y = 0.0;

    int nocnt = 0;

private:

};


#endif // OBSTACLE_H
