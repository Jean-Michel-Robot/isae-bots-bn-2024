
#include "Elevator.hpp"


Elevator::Elevator() :
    m_stepper(MOTOR_INTERFACE_TYPE, ELEV_DIR_PIN, ELEV_STEP_PIN),
    m_up_bumper(ELEV_BUMP_UP_PIN,ELEV_BUMP_TAU, ELEV_BUMP_THR, true),
    m_down_bumper(ELEV_BUMP_DOWN_PIN,ELEV_BUMP_TAU, ELEV_BUMP_THR, true){

        for(int i=0; i<9; i++){
            m_positions_step[i] = i*ELEV_STEP_DIFF;
        }    

}

void Elevator::setState(ElevatorState state, int sub_state){
    if(state == ElevatorState::MOVING){
        if(0 <= sub_state && sub_state < 9){
            m_stepper.moveTo(m_positions_step[sub_state]); //is this enough ??
            // m_stepper.setSpeed(ELEV_STEP_SPEED);
            m_state = state;
            m_sub_state = sub_state;      
        }
        else return;
    }

    else if(state == ElevatorState::RECAL_DOWN){
        m_stepper.setSpeed(ELEV_STEP_SPEED/3);
    }
    else if(state == ElevatorState::IDLE){
        m_state = state;
    }
}

void Elevator::setup(){

    m_down_bumper.setup();
    m_up_bumper.setup();

    this->setState(ElevatorState::IDLE, 0);
}

int Elevator::loop(){

    m_down_bumper.loop();
    m_up_bumper.loop();
    boolean stepper_bool = m_stepper.run(); //is false when the motor reaches the target position

    if(m_state == ElevatorState::MOVING){
        if(stepper_bool == 0){
            m_state = ElevatorState::IDLE;
            return 1;
        }
        else if(m_down_bumper.isSwitchPressed()){
            m_stepper.stop();
            m_state = ElevatorState::IDLE;
            return 2;
        }
        else if(m_up_bumper.isSwitchPressed()){
            m_stepper.stop();
            m_state = ElevatorState::IDLE;
            return 3;
        }
    }

    else if(m_state == ElevatorState::RECAL_DOWN){
        if(m_down_bumper.isSwitchPressed()){
            m_state = ElevatorState::IDLE;
            // for(int i=0; i<9; i++){
            //     m_positions_step[i] = m_stepper.currentPosition() + i*ELEV_STEP_DIFF;
            // }
            m_stepper.stop();
            m_stepper.setCurrentPosition(m_stepper.currentPosition());
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
    m_p_elevator->setState(ElevatorState::MOVING,stateVal.data);

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

