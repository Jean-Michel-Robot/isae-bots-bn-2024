#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;

std_msgs::Int16MultiArray feedbackMsg; //[motorId, feedbackId]
ros::Publisher feedbackSteppers("feedbackSteppers", &feedbackMsg);

//callback function, sets the target position of the selected motor
void stepperCb(const std_msgs::Int16MultiArray& cmdMsg);
void stepperCb(const std_msgs::Int16MultiArray& cmdMsg) {
  int targetPos = cmdMsg.data[1];
  if (targetPos != LOW_POS && targetPos != MID_POS && targetPos != HIGH_POS) feedbackMsg.data[1] = -1; //feedback -1
  else {
    setTargetPos(targetPos);
    feedbackMsg.data[1] = 1; //feedback 1
  }
  feedbackSteppers.publish(&feedbackMsg); //publishes the feedback msg
}

ros::Subscriber<std_msgs::Int16MultiArray> cmdSteppers("cmdSteppers", stepperCb);

void setupRos() {
  nh.initNode();
  nh.advertise(feedbackSteppers);
  nh.subscribe(cmdSteppers);

  feedbackMsg.data_length = 2; //configures length of the feedback msg
  feedbackMsg.data[0] = 0; //doesn't work, displays 156
}

void loopRos() {
  nh.spinOnce();
}
