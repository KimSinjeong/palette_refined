#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
#define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define DEVICE_NAME ""
#endif

const int LED=13;
const uint8_t DXID[4] = {1, 2, 3, 4};
const int BAUDRATE = 57600;
const uint8_t ACC[4] = {30, 30, 30, 20};
const uint8_t ANG[4] = {10, 10, 10, 5};
const char *LOG[4];

int32_t get_data[4] = {0, };

void move(const std_msgs::UInt16MultiArray&);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16MultiArray> sub("/arm/control_signal", move);
DynamixelWorkbench dx[4];

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // To know whether the setting is finished.
  
  bool result = true;
  uint16_t model_number[4] = {0, };

  for (int i = 0; i < 4; i++)
    result &= dx[i].init(DEVICE_NAME, BAUDRATE, LOG+i);

  for (int i = 0; i < 4; i++)
    result &= dx[i].ping(DXID[i], model_number+i, LOG+i);

  // jointMode 주의
  // 각가속도 0으로 하면 무제한, 처음에는 낮게 설정해 테스트 하는 것이 좋음.
  // 각속도 100으로 하면 존나빠름 - 그 상태로 제한 넘어가면 리셋해야 함
  for (int i = 0; i < 4; i++)
    result &= dx[i].jointMode(DXID[0], ACC[i], ANG[i], LOG+i); // ID, 각속도, 각가속도, 에러 로그 포인터

  // ROS
  nh.initNode();
  nh.subscribe(sub);
  digitalWrite(LED, LOW); // If the LED is turned off, the setting is finished.
}

void loop() {
  nh.spinOnce();
  delay(1);
}

// Handler
void move(const std_msgs::UInt16MultiArray& joint)
{
  //need to error pruning for exception
  if (0 < joint.data[0] && joint.data[0] < 1024
   && 900 < joint.data[1] && joint.data[1] < 3100
   && 800 < joint.data[2] && joint.data[2] < 3200
   && 200 < joint.data[3] && joint.data[3] < 800)
  {
    for (int i = 0; i < 4; i++)
      dx[i].goalPosition(DXID[i], (int32_t)joint.data[i]);
  
    for (int i = 0; i < 4; i++)
      dx[i].itemRead(DXID[i], "Present_Position", get_data+i, LOG+i);
  }
}
