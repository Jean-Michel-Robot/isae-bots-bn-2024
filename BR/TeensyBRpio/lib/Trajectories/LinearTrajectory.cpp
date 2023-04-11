/*
   
*/

#include "LinearTrajectory.hpp"

LinearTrajectory::LinearTrajectory(uint32_t t_0, float a, float b)
{
    this->t_0 = t_0;
    this->a = a;
    this->b = b;

}


Position2D LinearTrajectory::getPointAtTime(uint32_t current_time)  //TODO general class for this 
{

}