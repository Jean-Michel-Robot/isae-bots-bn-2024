#ifndef _ROS_CALLBACK_HPP_
#define _ROS_CALLBACK_HPP_

enum AsservCallback {
    OK_POS = 1,
    OK_TURN = 2,
    OK_REVERSE = 3,

    OK_READY = 5,
    OK_IDLE = 6,

    ERROR_ASSERV = 0,
};

#endif