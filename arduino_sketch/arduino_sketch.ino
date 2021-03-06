/***************************************
*                                      *
*   Observer Feedback Control System   *
*                                      *
****************************************/

 float r = 0.5;
 float Ts = 0.01;
 float tt = 0;
 float current_time, previous_time, elapsed_time;

 long count = 0;
 
 float x1_hat = 0;
 float x2_hat = 0;
 
 float u = 0;
 float y = 0;
 int dutyCycle  = 0;

 float a11 = -29;
 float a12 = -1721;
 float a21 = 1;
 float a22 = -51;

 float b11 = 1;
 float b12 = 22.76;
 float b21 = 0;
 float b22 = 0.675;

 float k1 = -9;
 float k2 = 500;
 float g = 6.614;

void setup(){
  
  pinMode(5,OUTPUT); 
  TCCR2B=(TCCR2B&0xF8) | 2;
  pinMode(3,OUTPUT);
  Serial.begin(9600);
}    
 
void loop(){

  //current_time = millis(); // get current time
  
  // design a squarewave reference signal
  count = count +1;
  if(count == 100){
    r = -r;
    count = 0;
  }

  // read analog feedback signal
  y = analogRead(A1);
  
  // range for y is -1...6V
  y = (float)(7*y/1023-1);
  
  // develop states of the controller
  x1_hat = x1_hat + Ts*(a11*x1_hat+a12*x2_hat+b11*u+b12*y);
  x2_hat = x2_hat + Ts*(a21*x1_hat+a22*x2_hat+b21*u+b22*y);

  // control signal
  u = -(k1*x1_hat+k2*x2_hat)+g*r;
  u = bound(u,-15,15);
  
  // design a pwm output
  // -15V -> 0
  // 15V -> 255
  dutyCycle=round(255/30*u+255/2);
  
  analogWrite(3,dutyCycle);
  Serial.print(x1_hat*4.2);// velocity
  Serial.print('\t');
  Serial.print(x2_hat*75);//displacement
  Serial.print('\n');

  //Serial.print("\t\t");
  //Serial.print(tt);
  //Serial.print('\n');
  delay(10);
  //elapsed_time = millis() - current_time;
  //Serial.print("\t\telapsed_time = "); Serial.print(elapsed_time);
  //Serial.print("\t\tserial port time = "); Serial.print(millis()-current_time);
  //Serial.print("\n");
}
           
float bound(float x, float x_min, float x_max){
  if(x < x_min){x = x_min;}
  if(x > x_max){x = x_max;}
  return x;
}

// end



