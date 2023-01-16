#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;

int feedbackArray[2]; //[stepperId, feedbackId]
std_msgs::Int16MultiArray feedbackMsg;
ros::Publisher feedbackSteppers("feedbackSteppers", &feedbackMsg);

//feedback function, sends feedback on feedbackSteppers topic
void steppersFb(int feedbackArray[]) {
  feedbackMsg.data = feedbackArray;
  feedbackSteppers.publish(&feedbackMsg); //publishes the feedback msg
}

//callback function, sets the target position of the selected motor
void steppersCb(const std_msgs::Int16MultiArray& cmdMsg);
void steppersCb(const std_msgs::Int16MultiArray& cmdMsg) {
  int stepperId = cmdMsg.data[0];
  int targetPos = cmdMsg.data[1];
  
  if (stepperId != FRONT && stepperId != BACK) {
    feedbackArray[0] = -1; //wrong stepper number
    feedbackArray[1] = 63; //random value, meaningless in this case
  }
  else {
    feedbackArray[0] = steppers[stepperId].getId(); //id of the stepper
    
    if (targetPos != LOW_POS && targetPos != MID_POS && targetPos != HIGH_POS) {
      feedbackArray[1] = -1; //wrong position
    }
    else {
      steppers[stepperId].setTargetPos(targetPos); //setting target position of stepper
      feedbackArray[1] = 1; //allowed position, command sent
    }
      
  }

  steppersFb(feedbackArray);
}

ros::Subscriber<std_msgs::Int16MultiArray> cmdSteppers("cmdSteppers", steppersCb);

void setupRos() {
  nh.initNode();
  nh.advertise(feedbackSteppers);
  nh.subscribe(cmdSteppers);

  feedbackMsg.data_length = 2; //configures length of the feedback msg
}

void loopRos() {
  nh.spinOnce();
}
