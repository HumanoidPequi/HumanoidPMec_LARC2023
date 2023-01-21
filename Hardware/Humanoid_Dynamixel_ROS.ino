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

#define timeMS 1    // tempo de publicação entre os motores

// define id dos motores

//perna direita
#define rYawHip     1
#define rRollHip    2
#define rPitchHip   3
#define rKnee       4
#define rAnklePitch 5
#define rAnkleRoll  6

//perna esquerda
#define lYawHip     7
#define lRollHip    8
#define lPitchHip   9
#define lKnee       10
#define lAnklePitch 11
#define lAnkleRoll  12

//braço direito
#define rSholder    13
#define rElbow      14
#define rHand       15

//braço esquerdo
#define lSholder    16
#define lElbow      17
#define lHand       18

DynamixelWorkbench dxl_wb;

#include<stdio.h>
std_msgs::String msg;
ros::Publisher pub("arm/position", &msg);

#define JOINT_NUMBER 18

//Inicio ROSSERIAL
ros::NodeHandle  nh;

//Constantes
uint8_t armJoints[JOINT_NUMBER] = {rYawHip, rRollHop, rPitchHip, rKnee, rAnklePich, rAnkleRoll, lYawHip, lRollHop, lPitchHip, lKnee, lAnklePich, lAnkleRoll, rSholder, rElbow, rHand, lSholder, lElbow, lHand};
bool reversedJoints[JOINT_NUMBER] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}; //joints where the angle increse has oposite signal to the rviz model
unsigned int basePos[JOINT_NUMBER] = {2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047}; // angulos iniciais  (12Bits) (0 ~ 4096)
unsigned int minPos[JOINT_NUMBER] = {1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705, 1705}; // angulos minimos (12Bits)
unsigned int maxPos[JOINT_NUMBER] = {2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388, 2388}; // angulos máximos (12Bits)

//Variáveis
int posAng[JOINT_NUMBER] = {0}; // angulos de -1800 a 1800 (16Bits ~ rosserial)
unsigned int posXYZ[JOINT_NUMBER] = {2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047}; // mesmos angulos iniciais (12Bits)
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
  sprintf(str, "%d, %d, %d, %d, %d, %d", posReal[0], posReal[1], posReal[2], posReal[3], posReal[4], posReal[5]);
  msg.data = str;
  delay(1);
  pub.publish(&msg);
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("arm/cmd_position", callback);

//Fim ROSSERIAL

void setup() {
  //Configure all basic setting
  Led_Setup();
  Button_Setup();
  dynamixelSetup();
  // Baud rate rosserial
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop() {
  nh.spinOnce();
  if (digitalRead(BOARD_BUTTON_PIN)) {
    dxl_wb.torqueOff(254);  //Broadcast de torqueOff caso o botão esteja pressionado
  }
    control();
    feedback();
}

//== Setup function ==
void dynamixelSetup(void) {
  dxl_wb.init(DEVICE_NAME, BAUDRATE);  //Inicializa a placa
  for (int i = 0; i < JOINT_NUMBER; i++) {
    dxl_wb.jointMode(JOINT_NUMBER, 0, 0); //Configura o motor para o modo junta
    delay(1);
  }
  control();
}

//Configure onboard led pin
void Led_Setup(void) {
  pinMode(14, OUTPUT);
}

//Configure onboard button pin
void Button_Setup(void) {
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
}

void control() {
  for (int i = 0; i < JOINT_NUMBER; i++) {
    dxl_wb.goalPosition(armJoints[i], (int32_t)posXYZ[i]);
    delay(timeMS);
  }
  if (ledStatus == true) {
    digitalWrite(14, HIGH);
  }
  else {
    digitalWrite(14, LOW);
  }
  ledStatus = !ledStatus;
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
