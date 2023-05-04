
#include "Elevator.hpp"


Elevator::Elevator() :
    m_stepper(MOTOR_INTERFACE_TYPE, ELEV_DIR_PIN, ELEV_STEP_PIN){

        for(int i=0; i<9; i++){
            m_positions_step[i] = ELEV_STEP_ZERO + ELEV_STEP_DIFF;
        }

        this->setState(0);    

}

void Elevator::setState(int state){
    if(0 <= state && state < 9){
        m_stepper.moveTo(m_positions_step[state]); //is this enough ??
        m_state = state;
    }
}

void Elevator::loop(){
    ;
}

ElevatorROS::ElevatorROS(Elevator* p_elevator, ros::NodeHandle* p_nh) : 
    m_sub("/strat/elevator", subCallback){
        m_p_elevator = p_elevator;
        m_p_nh = p_nh;

        m_p_nh->subscribe(m_sub);

}

void ElevatorROS::subCallback(const std_msgs::Int16& stateVal){
    m_p_elevator->setState(stateVal.data);
}

