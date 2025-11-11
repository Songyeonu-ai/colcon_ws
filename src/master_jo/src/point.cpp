#include "../include/master_jo/point.hpp"

Point::Point(int x, int y)
{
    this->px = x;
    this->py = y;
}

bool Point::setX(int x)
{
    this->px = x;
    return true;
}

bool Point::setY(int y)
{
    this->py = y;
    return true;
}

int Point::x()
{
    return this->px;
}

int Point::y()
{
    return this->py;
}

Point Point::operator+(Point op2)
{
    Point tmp;
    tmp.px = this->px + op2.px;
    tmp.py = this->py + op2.py;

    return tmp;
}

Point Point::operator-(Point op2)
{
    Point tmp;
    tmp.px = this->px - op2.px;
    tmp.py = this->py - op2.py;

    return tmp;
}

Point &Point::operator+=(Point op2)
{
    this->setX(this->x() + op2.x());
    this->setY(this->y() + op2.y());

    return *this;
}

Point &Point::operator-=(Point op2)
{
    this->setX(this->x() - op2.x());
    this->setY(this->y() - op2.y());

    return *this;
}
