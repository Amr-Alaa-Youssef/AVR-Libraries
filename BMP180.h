#ifndef BMP180_H
#define BMP180_H
#include<Wire.h>
class BMP180{
  private:
    const uint8_t I2C_Address=0x77;
    const uint8_t Control_Register=0xF4;
    const uint8_t Temperature_Command=0x2E;
    const uint8_t Pressure_Command=0xB4;
    const uint8_t Data_MSB=0xF6;
    const uint8_t Data_LSB=0xF7;
    const uint8_t Data_XLSB=0xF8;
    int16_t AC1,AC2,AC3,CB1,CB2,MB,MC,MD;
    uint16_t AC4,AC5,AC6;
    int32_t X1,X2,X3,B3,B5,B6;
    uint32_t B4,B7;
    uint32_t Temperature_Request_Time,Pressure_Request_Time;
    int8_t Current_Step=0;
    int32_t Uncompensated_Pressure=0;
    float Pressure=0.0;
    float Temperature=0.0;
    bool Error_Flag;
    bool Is_Initialized=false;
    float Filtered_Pressure=0.0;
    float Filtered_Temperature=0.0;
    void Send_Command(uint8_t MSB_Address);
    void Send_Command(uint8_t MSB_Address,uint8_t LSB_Address);
    int16_t Request_2_Bytes();
    int16_t Read_2_Registers(uint8_t MSB_Address);
    int32_t Request_3_Bytes();
    void Step_0();
    void Step_1();
    void Step_2();
    void Step_3();
    void Filter_Values(float Cutoff_Frequency);
  public:
    BMP180();
    void Initialize();
    float Get_Temperature();
    float Get_Pressure();
    void Request_Measurements();
};
void BMP180::Send_Command(uint8_t MSB_Address){
  Wire.beginTransmission(I2C_Address);
  Wire.write(MSB_Address);
  Wire.endTransmission();
}
void BMP180::Send_Command(uint8_t MSB_Address,uint8_t LSB_Address){
  Wire.beginTransmission(I2C_Address);
  Wire.write(MSB_Address);
  Wire.write(LSB_Address);
  Wire.endTransmission();
}
int16_t BMP180::Request_2_Bytes(){
  if(Wire.requestFrom(I2C_Address,uint8_t(2))!=2){
    Error_Flag=true;
    return 0;
  }
  int16_t Return_Value=(Wire.read()<<8)|Wire.read();
  return Return_Value;
}
int32_t BMP180::Request_3_Bytes(){
  if(Wire.requestFrom(I2C_Address,uint8_t(3))!=3){
    Error_Flag=true;
    return 0;
  }
  int32_t Return_Value=(((((((int32_t)Wire.read())<<8)+Wire.read())<<8)+Wire.read())>>6);
  return Return_Value;
}
int16_t BMP180::Read_2_Registers(uint8_t MSB_Address){
  Send_Command(MSB_Address);
  return Request_2_Bytes();
}
BMP180::BMP180(){}
void BMP180::Initialize(){
  Wire.begin();
  Error_Flag=false;
  AC1=Read_2_Registers(0xAA);
  AC2=Read_2_Registers(0xAC);
  AC3=Read_2_Registers(0xAE);
  AC4=(uint16_t)Read_2_Registers(0xB0);
  AC5=(uint16_t)Read_2_Registers(0xB2);
  AC6=(uint16_t)Read_2_Registers(0xB4);
  CB1=Read_2_Registers(0xB6);
  CB2=Read_2_Registers(0xB8);
  MB=Read_2_Registers(0xBA);
  MC=Read_2_Registers(0xBC);
  MD=Read_2_Registers(0xBE);
  if(Error_Flag){return;}
  Is_Initialized=true;
}
float BMP180::Get_Temperature(){
  return Filtered_Temperature;
}
float BMP180::Get_Pressure(){
  return Filtered_Pressure;
}
void BMP180::Request_Measurements(){
  if(Current_Step==0){
    Step_0();
  }else if(Current_Step==1){
    Step_1();
  }else if(Current_Step==2){
    Step_2();
  }else if(Current_Step==3){
    Step_3();
  }
}
void BMP180::Step_0(){
  Send_Command(Control_Register,Temperature_Command);
  Temperature_Request_Time=micros();
  Current_Step=1;
}
void BMP180::Step_1(){
  if((micros()-Temperature_Request_Time)<5000){return;}
  Current_Step=2;
  Send_Command(Data_MSB);
  int32_t Uncompensated_Temperature=(int32_t)Request_2_Bytes();
  if(Error_Flag){return;}
  X1=(Uncompensated_Temperature-(int32_t)AC6)*((int32_t)AC5)>>15;
  X2=((int32_t)MC<<11)/(X1+(int32_t)MD);
  B5=X1+X2;
  int32_t Integer_Temperature=(B5+8)>>4;
  Temperature=float(Integer_Temperature)*0.1;
}
void BMP180::Step_2(){
  Send_Command(Control_Register,Pressure_Command);
  Pressure_Request_Time=micros();
  Current_Step=3;
}
void BMP180::Step_3(){
  if((micros()-Pressure_Request_Time)<15000){return;}
  Current_Step=0;
  Send_Command(Data_MSB);
  Uncompensated_Pressure=Request_3_Bytes();
  if(Error_Flag){return;}
  B6=B5-4000;
  X1=(CB2*((B6*B6)>>12))>>11;
  X2=(AC2*B6)>>11;
  X3=X1+X2;
  B3=(((((int32_t)AC1)*4+X3)<<2)+2)>>2;
  X1=(AC3*B6)>>13;
  X2=(CB1*((B6*B6)>>12))>>16;
  X3=((X1+X2)+2)>>2;
  B4=(AC4*(uint32_t)(X3+32768))>>15;
  B7=((uint32_t)(Uncompensated_Pressure-B3)*(50000>>2));
  int32_t Integer_Pressure=(B7<0x80000000)?((B7<<1)/B4):((B7/B4)<<1);
  X1=(Integer_Pressure>>8)*(Integer_Pressure>>8);
  X1=(X1*3038)>>16;
  X2=(-7357*Integer_Pressure)>>16;
  Integer_Pressure+=((X1+X2+3791)>>4);
  Pressure=float(Integer_Pressure);
  Filter_Values(1.0);
}
void BMP180::Filter_Values(float Cutoff_Frequency){
  static uint32_t Previous_Call_Time=micros();
  uint32_t Current_Call_Time=micros();
  float Time_Constant=159154.943/Cutoff_Frequency;
  float Exponential=exp(-1.0*float(Current_Call_Time-Previous_Call_Time)/Time_Constant);
  Filtered_Pressure=(1.0-Exponential)*Pressure+Exponential*Filtered_Pressure;
  Filtered_Temperature=(1.0-Exponential)*Temperature+Exponential*Filtered_Temperature;
  Previous_Call_Time=Current_Call_Time;
}
#endif
