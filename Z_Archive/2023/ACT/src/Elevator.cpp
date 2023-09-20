
#include "Elevator.hpp"


Elevator::Elevator() :
    m_stepper(MOTOR_INTERFACE_TYPE, ELEV_DIR_PIN, ELEV_STEP_PIN){

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
        else return;
    }

    else if(state == ElevatorState::IDLE){
        m_state = state;
    }
}

void Elevator::setZeroPosition(){
    m_stepper.setCurrentPosition(0);
    m_sub_state = 0;
}

void Elevator::setup(){

    m_stepper.setCurrentPosition(0);

    this->setState(ElevatorState::IDLE, 0);

    m_stepper.setAcceleration(50);
    m_stepper.setMaxSpeed((float)ELEV_STEP_SPEED);
}

int Elevator::loop(){

    boolean stepper_bool = m_stepper.run(); //is false when the motor reaches the target position

    if(m_state == ElevatorState::MOVING){
        if(stepper_bool == 0){
            m_state = ElevatorState::IDLE;
            return 1;
        }
    }

    return 0;
}

Elevator* ElevatorROS::m_p_elevator = NULL;
ros::NodeHandle* ElevatorROS::m_p_nh = NULL;

ElevatorROS::ElevatorROS(Elevator* p_elevator, ros::NodeHandle* p_nh) : 
    m_sub("/strat/elevator", subCallback),
    m_pub_feedback("/strat/elevator_feedback", &(ElevatorROS::m_msg_feedback)){
        m_p_elevator = p_elevator;
        m_p_nh = p_nh;
}

void ElevatorROS::subCallback(const std_msgs::Int16& stateVal){
    m_p_nh->loginfo("[ELEVATOR] Order");
    int order_id = stateVal.data;
    if(order_id >= 0 && order_id <= 8){
        m_p_elevator->setState(ElevatorState::MOVING,stateVal.data);
    }
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

