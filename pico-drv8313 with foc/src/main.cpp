#include <Arduino.h>
#include <SimpleFOC.h>
#include "Wire.h"
//设置I2C设备地址，每个电机设置一个地址
#define I2C_DEV_ADDR 0x29
//电机转速
String target_velocity_string;
int target_velocity = 0;
//主机请求回复
void onRequest(){
  Wire1.print(target_velocity);
  //Serial.println("HaveRequest");
}
//收包中断，收到速度修改电机转动速度
void onReceive(int len){
  //Serial.printf("Receive[%d]: ", len);
  while(Wire1.available()){
    target_velocity_string = Wire1.read();
    target_velocity = target_velocity_string.toInt();
    if(target_velocity>=128) target_velocity = target_velocity - 256;
  }
}

//编码器采用I2C通信
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
//电机极对数
BLDCMotor motor = BLDCMotor(8);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(19, 21, 22, 23);


void setup() {
  Serial.begin(115200);
  //检查各个部分是否正确
  Serial.setDebugOutput(true);
  Wire1.onReceive(onReceive);
  Wire1.onRequest(onRequest);
  //与主机进行通信的I2C引脚
  Wire1.begin((uint8_t)I2C_DEV_ADDR,13,15);


  motor.useMonitoring(Serial);
  //与编码器通信的I2C引脚
  Wire.setPins(25,26);
  Wire.setClock(400000);
  sensor.init();
  motor.linkSensor(&sensor);
  //供电电压
  driver.voltage_power_supply = 16.8;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 3;   // [V]
  motor.controller = MotionControlType::velocity;
  //PID参数
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 2;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01;
  motor.init();
  motor.initFOC();
  motor.monitor_variables = _MON_ANGLE; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE
  _delay(1000);

  // #if CONFIG_IDF_TARGET_ESP32
  // char message[64];
  // snprintf(message, 64, "%u Packets.", target_velocity);
  // Wire1.slaveWrite((uint8_t *)message, strlen(message));
  // #endif
  Serial.print(1);
}

void loop() {
  //闭环速度控制
  motor.loopFOC();
  motor.move(target_velocity);
  
  if(sensor.getVelocity()>3){
    motor.voltage_limit = 16.5;
  }
  // Serial.println(target_velocity);
  // motor.monitor();
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
  
}
