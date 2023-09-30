/*
  * Elevator.cpp
  * Elevator class
  * -> May 2023
  * -> Sept. 2023 : more comments
*/

#include "Elevator.hpp"


Elevator::Elevator() :
    m_stepper(MOTOR_INTERFACE_TYPE, ELEV_DIR_PIN, ELEV_STEP_PIN){

        // Initialization of the number of steps for each position
        for(int i=0; i<9; i++){
            m_positions_step[i] = i*ELEV_STEP_DIFF;
        }    

}

void Elevator::setState(ElevatorState state, int sub_state){
    if(state == ElevatorState::MOVING){
        if(0 <= sub_state && sub_state < 9){
            m_stepper.moveTo(m_positions_step[sub_state]);
            m_state = state;
            m_sub_state = sub_state;      
        }
        else return; //invalid position, do nothing
    }

    else if(state == ElevatorState::IDLE){
        m_state = state;
    }
}


void Elevator::setZeroPosition(){
    m_stepper.setCurrentPosition(0); //method to reset the number of steps of the motor
    m_sub_state = 0;
}


void Elevator::setup(){

    // Setting the position seen by the stepper to '0 step'
    m_stepper.setCurrentPosition(0);

    // Set the state to IDLE and position 0
    this->setState(ElevatorState::IDLE, 0);

    // Set the acceleration of the stepper
    m_stepper.setAcceleration(50);

    // Set the max speed of the stepper
    m_stepper.setMaxSpeed((float)ELEV_STEP_SPEED);
}

int Elevator::loop(){

    // Send a step instruction to the stepper if needed
    boolean stepper_bool = m_stepper.run(); //is false when the motor reaches the target position

    // Set state to IDLE if the current state is MOVING and there si no pmore step to do (i.e. stepper_bool = false)
    if(m_state == ElevatorState::MOVING){
        if(stepper_bool == 0){
            m_state = ElevatorState::IDLE;
            return 1;
        }
    }

    return 0;
}

// STATIC varialble must be explicitely declared (and outside of a constructor)
Elevator* ElevatorROS::m_p_elevator = NULL;
ros::NodeHandle* ElevatorROS::m_p_nh = NULL;


ElevatorROS::ElevatorROS(Elevator* p_elevator, ros::NodeHandle* p_nh) : 

    //ROS subscriber constructor : 
    //  "/strat/elevator" is the name of the topic
    //  subCallback : the callback fucntion
    m_sub("/strat/elevator", subCallback),

    //ROS publisher constructor : 
    //  "/strat/elevator_feedback is the name of the topic
    //  &(ElevatorROS::m_msg_feedback) is the pointer to the ROS message that is inted to be sent through the subscriber
    m_pub_feedback("/strat/elevator_feedback", &(ElevatorROS::m_msg_feedback)){

        m_p_elevator = p_elevator;
        m_p_nh = p_nh;
}


void ElevatorROS::subCallback(const std_msgs::Int16& stateVal){

    // Logging that we received the order
    m_p_nh->loginfo("[ELEVATOR] Order");

    // Extrating the content of the message (here an integer)
    int order_id = stateVal.data;

    // If the message is between 0 and 8, move to that position
    if(order_id >= 0 && order_id <= 8){
        m_p_elevator->setState(ElevatorState::MOVING,stateVal.data);
    }

    // If the message -1, reset the number of steps to 0
    else if(order_id == -1){
        m_p_elevator->setZeroPosition();
    }
    

}


void ElevatorROS::setup(){
    m_p_nh->subscribe(m_sub);
    m_p_nh->advertise(m_pub_feedback);
    m_p_elevator->setup();

    m_p_nh->loginfo("[ELEVATOR] Setup");
}

void ElevatorROS::loop(){

    int loop_ret = m_p_elevator->loop();

    if(loop_ret!=0){
        m_msg_feedback.data = loop_ret;
        m_pub_feedback.publish(&m_msg_feedback);
    }
}

