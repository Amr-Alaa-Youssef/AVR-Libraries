//Include needed headers
#include<SPI.h>
//Declare structures and enumerators
struct MAX6675{
  float _Temperature;
  uint8_t _Pin;
  uint8_t _Initialized;
  uint32_t Previous_Conversion_Time;
  uint8_t First_Reading;
};
typedef struct MAX6675 MAX6675;
enum MAX6675_Error{
  MAX6675_OK,
  MAX6675_NO_CONNECTION,
  MAX6675_NO_NEW_DATA,
  MAX6675_NOT_INITIALIZED
};
//Functions prototypes
void MAX6675_Initialize(MAX6675 *Thermocouple,uint8_t Pin);
MAX6675_Error MAX6675_Request_Measurment(MAX6675 *Thermocouple);
float MAX6675_Get_Temperature(MAX6675 *Thermocouple);
//Functions implementation
void MAX6675_Initialize(MAX6675 *Thermocouple,uint8_t Pin){
  static uint8_t SPI_Initialized=0;
  if(SPI_Initialized==0){
    SPI.begin();
    SPI_Initialized=1;
  }
  Thermocouple->_Pin=Pin;
  digitalWrite(Pin,HIGH);
  pinMode(Pin,OUTPUT);
  Thermocouple->First_Reading=1;
  Thermocouple->_Initialized=1;
}
MAX6675_Error MAX6675_Request_Measurment(MAX6675 *Thermocouple){
  if(Thermocouple->_Initialized==0){return MAX6675_NOT_INITIALIZED;}
  uint32_t Currrent_Time=millis();
  if((Currrent_Time-Thermocouple->Previous_Conversion_Time)<220){return MAX6675_NO_NEW_DATA;}
  Thermocouple->Previous_Conversion_Time=Currrent_Time;
  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
  uint16_t Raw_Data=0;
  digitalWrite(Thermocouple->_Pin,LOW);
  Raw_Data=SPI.transfer(0);
  Raw_Data<<=8;
  Raw_Data+=SPI.transfer(0);
  digitalWrite(Thermocouple->_Pin,HIGH);
  SPI.endTransaction();
  if((Raw_Data&0x0004)!=0){return MAX6675_NO_CONNECTION;}
  Raw_Data>>=3;
  if(Thermocouple->First_Reading==1){
    Thermocouple->First_Reading=0;
    Thermocouple->_Temperature=(float)(Raw_Data&0x1FFF)*0.25;
    return MAX6675_OK;
  }
  Thermocouple->_Temperature=0.05*(float)(Raw_Data&0x1FFF)*0.25+0.95*Thermocouple->_Temperature;
  return MAX6675_OK;
}
float MAX6675_Get_Temperature(MAX6675 *Thermocouple){
  return Thermocouple->_Temperature;
}
