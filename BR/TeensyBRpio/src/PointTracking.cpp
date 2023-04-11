/*
   
*/

#include "PointTracking.hpp"

PointTracking::PointTracking()
{
    trackingPos = Position2D(0.0, 0.0, 0.0);
}


void PointTracking::calculatePointPos(uint32_t current_time)
{
    
}






Position2D PointTracking::getPointPos()
{
  return trackingPos;
}



