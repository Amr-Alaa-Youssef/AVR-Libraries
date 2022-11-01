#ifndef HONEYWELL_HSC
#define HONEYWELL_HSC
class Honeywell_HSC{
  private:
    float _Minimum_Pressure,_Maximum_Pressure,_Supply_Voltage;
    uint8_t _Sensor_Pin;
    uint32_t _Sampling_Time=1000;
    uint32_t Initialization_Time=3000000;
    float _Offset=0.0;
    float Actual_Differential_Pressure;
    float Output_Differential_Pressure;
    float Filter_Output_Value=0.0;
    float Filter_Reading(float Input_Value,float Cutoff_Frequency);
    bool _Is_Initialized=false;
    bool _First_Reading=true;
  public:
    Honeywell_HSC(uint8_t Sensor_Pin,float Minimum_Pressure,float Maximum_Pressure,float Supply_Voltage,uint32_t Sampling_Time);
    bool Is_Initialized();
    void Request_Measurments();
    float Get_Reading();
    void Tare();
};
Honeywell_HSC::Honeywell_HSC(uint8_t Sensor_Pin,float Minimum_Pressure,float Maximum_Pressure,float Supply_Voltage,uint32_t Sampling_Time){
  _Sensor_Pin=Sensor_Pin;
  _Supply_Voltage=(Supply_Voltage==0.0)?0.0:Supply_Voltage;
  _Minimum_Pressure=Minimum_Pressure;
  _Maximum_Pressure=(Maximum_Pressure==Minimum_Pressure)?(Minimum_Pressure+1.0):Maximum_Pressure;
  _Sampling_Time=(Sampling_Time>0)?Sampling_Time*1000:1000;
  pinMode(_Sensor_Pin,INPUT);
}
bool Honeywell_HSC::Is_Initialized(){
  return _Is_Initialized;
}
void Honeywell_HSC::Request_Measurments(){
  static uint32_t Previous_Call_Time=0;
  uint32_t Current_Call_Time=micros();
  if((Current_Call_Time-Previous_Call_Time)<_Sampling_Time){return;}
  Previous_Call_Time=Current_Call_Time;
  float Measured_Voltage=float(analogRead(_Sensor_Pin))*4.88758;
  float Unfiltered_Differential_Pressure=_Minimum_Pressure+(Measured_Voltage-0.1*_Supply_Voltage)/(0.8*_Supply_Voltage)*(_Maximum_Pressure-_Minimum_Pressure);
  if(_First_Reading){
    _First_Reading=false;
    Filter_Output_Value=Unfiltered_Differential_Pressure;
    return;
  }
  if(_Is_Initialized){
    Actual_Differential_Pressure=Filter_Reading(Unfiltered_Differential_Pressure,0.1);
  }else{
    Actual_Differential_Pressure=Filter_Reading(Unfiltered_Differential_Pressure,0.05);
    if(Current_Call_Time>=Initialization_Time){
      Tare();
      _Is_Initialized=true;
    }
  }
  Output_Differential_Pressure=Actual_Differential_Pressure-_Offset;
}
float Honeywell_HSC::Get_Reading(){
  return Output_Differential_Pressure;
}
void Honeywell_HSC::Tare(){
  _Offset=Actual_Differential_Pressure;
}
float Honeywell_HSC::Filter_Reading(float Input_Value,float Cutoff_Frequency){
  static uint32_t Previous_Call_Time=micros();
  uint32_t Current_Call_Time=micros();
  float Time_Constant=159154.943/Cutoff_Frequency;
  float Exponential=exp(-1.0*float(Current_Call_Time-Previous_Call_Time)/Time_Constant);
  Filter_Output_Value=(1.0-Exponential)*Input_Value+Exponential*Filter_Output_Value;
  Previous_Call_Time=Current_Call_Time;
  return Filter_Output_Value;
}
#endif
