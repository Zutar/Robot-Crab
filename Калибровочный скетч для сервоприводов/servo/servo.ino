#include <iarduino_MultiServo.h>                          
iarduino_MultiServo MSS;                                   

const int legs_pin[4][3] = {{4, 5, 6}, {12, 13, 14}, {0, 1, 2}, {8, 9, 10},  };

void setup(){                                               
   MSS.servoSet(1, SERVO_SG90);                                                     
   MSS.begin();                                             
}                                                           
                                                            
void loop(){              
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 3; j++){
      MSS.servoWrite(legs_pin[i][j],90);
      delay(250);     
    }  
  }                                         
}                                                          

