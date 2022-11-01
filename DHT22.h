#ifndef _DHT_22
#define _DHT_22
#define DHT_Ok 0
#define DHT_Timeout_Error 1
#define DHT_Acknowledge_Error 2
#define DHT_Connection_Error 3
#define DHT_Checksum_Error 4
#define DHT_Sampling_Time_Error 5
class DHT22{
  private:
    uint8_t _Sensor_Pin;
    float Temperature,Humidity;
    uint8_t Received_Data[5];
    uint32_t Previuos_Call_Time=0;
    uint8_t Last_Return=DHT_Ok;
    float Filtered_Humidity=0.0;
    float Filtered_Temperature=0.0;
    bool First_Call=true;
    bool Second_Call=true;
    uint8_t Return(uint8_t Return_Value);
    void Filter_Readings(float Cutoff_Frequency);
    bool _Data_Available=false;
  public:
    DHT22(uint8_t Sensor_Pin);
    uint8_t Request_Measurments();
    float Get_Temperature();
    float Get_Humidity();
    bool Data_Available();
};
uint8_t DHT22::Return(uint8_t Return_Value){
  Last_Return=Return_Value;
  return Return_Value;
}
void DHT22::Filter_Readings(float Cutoff_Frequency){
  static uint32_t Previous_Call_Time=micros();
  uint32_t Current_Call_Time=micros();
  float Time_Constant=159154.943/Cutoff_Frequency;
  float Exponential=exp(-1.0*float(Current_Call_Time-Previous_Call_Time)/Time_Constant);
  Filtered_Humidity=(1.0-Exponential)*Humidity+Exponential*Filtered_Humidity;
  Filtered_Temperature=(1.0-Exponential)*Temperature+Exponential*Filtered_Temperature;
  Previous_Call_Time=Current_Call_Time;
}
DHT22::DHT22(uint8_t Sensor_Pin){
  _Sensor_Pin=Sensor_Pin;
}
uint8_t DHT22::Request_Measurments(){
  uint32_t Current_Call_Time=millis();
  if((Current_Call_Time-Previuos_Call_Time)<2000){return Last_Return;}
  Previuos_Call_Time=Current_Call_Time;
  uint32_t Start_Time;
  pinMode(_Sensor_Pin,OUTPUT);
  digitalWrite(_Sensor_Pin,LOW);
  delayMicroseconds(1000);
  pinMode(_Sensor_Pin,INPUT);
  Start_Time=micros();
  while(digitalRead(_Sensor_Pin)==HIGH){
    if((micros()-Start_Time)>200){return Return(DHT_Timeout_Error);}
  }
  Start_Time=micros();
  while(digitalRead(_Sensor_Pin)==LOW){
    if((micros()-Start_Time)>100){return Return(DHT_Acknowledge_Error);}
  }
  Start_Time=micros();
  while(digitalRead(_Sensor_Pin)==HIGH){
    if((micros()-Start_Time)>100){return Return(DHT_Acknowledge_Error);}
  }
  for(uint8_t Current_Byte=0;Current_Byte<5;Current_Byte++){
    Received_Data[Current_Byte]=0;
    for(uint8_t Current_Bit=7;Current_Bit<8;Current_Bit--){
      Start_Time=micros();
      while(digitalRead(_Sensor_Pin)==LOW){
        if((micros()-Start_Time)>60){return Return(DHT_Connection_Error);}
      }
      Start_Time=micros();
      while(digitalRead(_Sensor_Pin)==HIGH){
        if((micros()-Start_Time)>80){return Return(DHT_Connection_Error);}
      }
      Received_Data[Current_Byte]|=((micros()-Start_Time)>40)?(1<<Current_Bit):0;
    }
  }
  uint8_t Checksum=Received_Data[0]+Received_Data[1]+Received_Data[2]+Received_Data[3];
  if(Received_Data[4]!=Checksum){return Return(DHT_Checksum_Error);}
  Received_Data[0]&=0x03;
  Received_Data[2]&=0x83;
  uint16_t Integer_Humidity=Received_Data[0];
  Integer_Humidity<<=8;
  Integer_Humidity+=Received_Data[1];
  Humidity=(float(Integer_Humidity)*0.1);
  uint16_t Integer_Temperature=Received_Data[2];
  Integer_Temperature<<=8;
  Integer_Temperature+=Received_Data[3];
  Temperature=(float(Integer_Temperature)*(((Received_Data[2]&0x80)==0x80)?-0.1:0.1));
  if((Humidity!=0.0)&&(Temperature!=0.0)&&(Second_Call==false)){_Data_Available=true;}
  if(First_Call){
    Filtered_Humidity=Humidity;
    Filtered_Temperature=Temperature;
    First_Call=false;
    return Return(DHT_Ok);
  }
  if(Second_Call){
    Filtered_Humidity=Humidity;
    Filtered_Temperature=Temperature;
    Second_Call=false;
    return Return(DHT_Ok);
  }
  
  Filter_Readings(0.2);
  return Return(DHT_Ok);
}
float DHT22::Get_Temperature(){return Filtered_Temperature;}
float DHT22::Get_Humidity(){return Filtered_Humidity;}
bool DHT22::Data_Available(){return _Data_Available;}
#endif
