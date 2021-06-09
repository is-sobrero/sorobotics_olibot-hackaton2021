/*
LICENZA
*/

#define FLOODING_THRESHOLD 10 

#include <Arduino_MKRIoTCarrier.h>
MKRIoTCarrier carrier;

float gyro_x;
float gyro_y;
float gyro_z;

typedef enum {
  STATE_IDLING, STATE_EARTHQUAKE, STATE_FIRE, STATE_FLOOD
} sysstate_t;

typedef enum {
  IDLE, EARTHQUAKE, FIRE, FLOOD
} eventtype_t;

int moistPin = A5;
void setup() {
  Serial.begin(115200);
  while(!Serial);
   CARRIER_CASE = false;
  carrier.begin();
  Serial.println("NO");

}

eventtype_t event;
sysstate_t state = STATE_IDLING;

float prec_gyro_x = 0;
float prec_gyro_y = 0;
float prec_gyro_z = 0;

#define GYRO_DERIV_THRESH 75
#define DERIV_EVENT_COUNT 20

uint8_t derivThresCount = 0;

eventtype_t getEvent(){
  float moistValue = map(analogRead(A5), 0, 1023, 100, 0);
  prec_gyro_x = gyro_x;
  prec_gyro_y = gyro_y;
  prec_gyro_z = gyro_z;

  carrier.IMUmodule.readGyroscope(gyro_x, gyro_y, gyro_z);
  
  float dgx_dt = gyro_x - prec_gyro_x;
  float dgy_dt = gyro_y - prec_gyro_y;
  float dgz_dt = gyro_z - prec_gyro_z;
  
  if (abs(dgx_dt) > GYRO_DERIV_THRESH) derivThresCount++;
  if (abs(dgy_dt) > GYRO_DERIV_THRESH) derivThresCount++;
  if (abs(dgz_dt) > GYRO_DERIV_THRESH) derivThresCount++;

  if (derivThresCount > DERIV_EVENT_COUNT){
    derivThresCount = 0;
    return EARTHQUAKE;
  }

  if (moistValue >= FLOODING_THRESHOLD)
    return FLOOD;  
  return IDLE;
}

void loop() {
  // macchina a stati del sistema
  switch(state){
    // Stato iniziale, controlla le condizioni di trigger del robot o dei soccorsi
    case STATE_IDLING:
      switch (getEvent()){
        case IDLE:
        break;
        case FLOOD:
          state = STATE_FLOOD;
          // chiama soccorsi
        break;
        case EARTHQUAKE:
          state = STATE_EARTHQUAKE;
          // chiama soccorsi
        break;
      }
    break;
    case STATE_FLOOD:
      Serial.println("ALLAGAMENTO");
    break;
    case STATE_EARTHQUAKE:
      Serial.println("TERREMOTO");
    break;
    default:
    break;
  }
}
