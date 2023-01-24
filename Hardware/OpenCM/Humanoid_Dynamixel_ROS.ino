//Pequi Mecânico 2022 luis.fernandoferreira@outlook.com
//Include Library
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16MultiArray.h>
#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
#define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define DEVICE_NAME ""
#endif

#define BAUDRATE  1000000

DynamixelWorkbench dxl_wb;

#include<stdio.h>
std_msgs::String msg;
ros::Publisher pub("marta/joints/position", &msg);

#define JOINT_NUMBER 12

//Inicio ROSSERIAL
ros::NodeHandle  nh;

//Constantes
//uint8_t armJoints[JOINT_NUMBER] = {9};//left leg
//uint8_t armJoints[JOINT_NUMBER] = {3,4,5,6};//left leg
//uint8_t armJoints[JOINT_NUMBER] = {9,10,11,12};//right leg
//uint8_t armJoints[JOINT_NUMBER] = {1,2,7,8};


uint8_t armJoints[JOINT_NUMBER] = {1,2,3,4,5,6,7,8,9,10,11,12};
//bool reversedJoints[JOINT_NUMBER]  = {false, false,false,false}; //joints where the angle increse has oposite signal to the rviz model
//bool reversedJoints[JOINT_NUMBER]  = {false}; //joints where the angle increse has oposite signal to the rviz model

bool reversedJoints[JOINT_NUMBER]  = {false, false,false,false,false, false,false,false,false,false,false,false}; //joints where the angle increse has oposite signal to the rviz model

unsigned int basePos[JOINT_NUMBER] = {2047,2047,2047,2047,2047,2047,2047,2047,2047,2047,2047,2047}; // angulos iniciais  (12Bits) (0 ~ 4096)

//unsigned int basePos[JOINT_NUMBER] = {2047,2047,2047,2047}; // angulos iniciais  (12Bits) (0 ~ 4096)
//unsigned int basePos[JOINT_NUMBER] = {2047}; // angulos iniciais  (12Bits) (0 ~ 4096)

unsigned int minPos[JOINT_NUMBER]  = {0,1705,1500,1705,1705,1705,1500,1705,0,1705,1705,1705}; // angulos minimos (12Bits)
//unsigned int minPos[JOINT_NUMBER]  = {0,1705,1500,1705}; // angulos minimos (12Bits)
//unsigned int minPos[JOINT_NUMBER]  = {0}; // angulos minimos (12Bits)
unsigned int maxPos[JOINT_NUMBER]  = {3000,2388,3000,2388,3000,3000,3000,3000,4096,3000,3000,3000}; // angulos máximos (12Bits)
//unsigned int maxPos[JOINT_NUMBER]  = {4096,2388,3000,2388}; // angulos máximos (12Bits)
//unsigned int maxPos[JOINT_NUMBER]  = {4096}; // angulos máximos (12Bits)

//Variáveis
int posAng[JOINT_NUMBER] = {0}; // angulos de -1800 a 1800 (16Bits ~ rosserial)
//unsigned int posXYZ[JOINT_NUMBER] = {4086, 1843, 1854, 1843}; // mesmos angulos iniciais (12Bits) left leg
//unsigned int posXYZ[JOINT_NUMBER] = {2050, 2044, 2446, 2047}; // mesmos angulos iniciais (12Bits) right leg
//unsigned int posXYZ[JOINT_NUMBER] = {1980, 2044, 1980, 2048}; // mesmos angulos iniciais (12Bits) hip

unsigned int posXYZ[JOINT_NUMBER] = {1980, 2044,4086, 1843, 1854, 1843,1980, 2048,2050, 2044, 2446, 2047}; // mesmos angulos iniciais (12Bits)
int32_t posReal[JOINT_NUMBER] = {0}; // angulos lidos do servo (12Bits)
bool ledStatus = true;

int convert(int ang_postiton, int base_position, bool joint_reversed) {
  //Converte os valores de 16 bits recebidos pelo rosserial para os valores de 12 bits
  int relative_position = map(ang_postiton, -1800, 1800, -2048, 2048);

  // Faz o offset com os angulos iniciais
  if (joint_reversed) {
    return base_position - relative_position;
  }
  return relative_position + base_position;
}

// Garante que as posições não passe dos valores máximos e mínimos
unsigned int constrained(unsigned int val, unsigned int out_min, unsigned int out_max ) {
  if (val < out_min) return out_min;
  if (val > out_max) return out_max;
  return val;
}

char str[100];

void callback(const std_msgs::Int16MultiArray& cmd_msg) {
  for (int i = 0; i < JOINT_NUMBER; i++) {
    posAng[i] = cmd_msg.data[i];
    posXYZ[i] = constrained(convert(posAng[i], basePos[i], reversedJoints[i]), minPos[i], maxPos[i]);
  }
  
  // Constoi mensagem para a publicada no ros 'arm/position' (debug)
  sprintf(str, "%d, %d, %d, %d", posReal[0], posReal[1], posReal[2],posReal[3]);
  msg.data = str;
  delay(1);
  pub.publish(&msg);
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("marta/joints/cmd_position", callback);

//Fim ROSSERIAL

//Configure onboard led pin
void Led_Setup(void) {
  pinMode(14, OUTPUT);
}

void control() {
  for (int i = 0; i < JOINT_NUMBER; i++) {
    dxl_wb.goalPosition(armJoints[i], (int32_t)posXYZ[i]);
    delay(1);
  }
  if (ledStatus == true) {
    digitalWrite(14, HIGH);
  }
  else {
    digitalWrite(14, LOW);
  }
  ledStatus = !ledStatus;

  feedback();
}

//== Setup function ==
void dynamixelSetup(void) {

  uint16_t model_number = 0;

  dxl_wb.init(DEVICE_NAME, BAUDRATE);
  for(int i = 0; i < JOINT_NUMBER;i++){
    dxl_wb.ping(armJoints[i], &model_number);
  }  

  for(int i = 0; i < JOINT_NUMBER; i++)
  {
    dxl_wb.jointMode(armJoints[i], 0, 0);
  }    
 
  control();
}



//Configure onboard button pin
void Button_Setup(void) {
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
}



void feedback() {
  int32_t get_data = 0;
  bool result = false;
  for (int i = 0; i < JOINT_NUMBER; i++) {
    result = dxl_wb.itemRead(armJoints[i], "Present_Position", &get_data);
    if (result == true)
    {
      posReal[i] = get_data;
    }
    delay(1);
  }
}

void setup() {
  //Configure all basic setting
  Led_Setup();
  Button_Setup();
  
  // Baud rate rosserial
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  dynamixelSetup();
}

void loop() {
  nh.spinOnce();
    control();
}
