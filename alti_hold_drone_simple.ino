 /*
 * recent changes - 
 * gyro scale changed from 500deg/s to 1000deg/s
 * 1.PID changed to EID, this system exponentially increases Kp with the error and Kd with rotation rate. 
 * 2.gyro scale changed from 250deg/s to 500 deg/s 
 * 3.limiter function added to limit the pwm output between 1100 and 2000 us
 * 4.on the fly PD tuning added (needs 5th channel from receiver to be connected to pin 12 on the arduino 
 * 5.max value of pitch and roll input increased to about 40 degrees(if you have pitch/roll input pwm ranging from 1000-2000us)
 * 6.max value of yaw input increased from 50deg/s to 500 deg/s
 * 7.IMAX changed from 1000 to 100 and Ki changed from 0.025 to 0.25
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define BaseKp 5.0
#define BaseKd 1.5
int tune; // tune requires channel 5 to be used for scaling PD up and down
float Kp_pitch,Kp_roll;
#define Ki 0.03  //premultiply the time with Ki 
#define IMAX 2000  //this is not in degrees, this is degrees*400 
#define YawKp 10   //Kp for yaw rate 
#define minValue 1100  //min throttle value, this is to prevent any motor from stopping mid air.
#define maxValue 2000 //max pwm for any motor 
#define YAW_MAX 300   //max value of yaw input
#define CONSTANT 0.0003 


unsigned long FL,FR,BL,BR; //(Front left, front right, back left, back right motor)
long esc_timer,end_timer;   //timers to keep track of width of the pulses
//pins being used ,FLpin=3,FRpin=4,BLpin=5,BRpin=6, standard code- B(7)(6)(5)(4)(3)(2)(1)(0). B is for binary 
#define FLpin B11110111   // binary codes for pulling a pin low. refer to motorWrite() function to understand what is happening 
#define FRpin B11101111   // the pins are set high at the begining of each loop
#define BLpin B11011111 
#define BRpin B10111111
#define PULL_HIGH B01111000 //binary number for pulling pins 3,4,5,6 high at the same time 
#define PULL_LOW B10000111  //binary number for pulling pins 3,4,5,6 low at the same time 

//-----ACCEL-GYRO STUFF BEGINS---------------------
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 pin high for when you have more than one accelerometers(not sure why you'd want that if you want a fast code but okay).
int16_t a[3];  //accelerations from mpu6050
int16_t g[3];  //gyration rates from mpu6050

float A[3],G[3],lastA[3]={0,0,0},lastG[3]={0,0,0},offsetA[3],offsetG[3],T[2]; //x=0,y=1,z=2, T=tilt.
float sigma[2]={0.0,0.0};  //variable for the integral part
int i,j;
bool connection;

//-----------ACCEL-GYRO STUFF ENDS----------------

//------variables for interrupt-------
volatile unsigned long timer[8];   
volatile byte last_channel[6]={0,0,0,0,0};
volatile int input[6]={1000,1500,1500,1500,1000,0}; //input variables to record the input PWM signals from receiver 
//---------------------------

void setup()
{
 // Serial.begin(250000);
  //enable interrupts on B port
  PCICR |= (1 << PCIE0);   
  PCMSK0 |= (1 << PCINT0); //8
  PCMSK0 |= (1 << PCINT1); //9
  PCMSK0 |= (1 << PCINT2); //10
  PCMSK0 |= (1 << PCINT3); //11
  PCMSK0 |= (1 << PCINT4); //12 for tuning PD 

  //enable interrupts on C port(Analog pins)
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8);

 
  //===========attaching motors=====================
  DDRD |= PULL_HIGH;//set pins 3,4,5,6 as output, PULL_HIGH here is only used to let the arduino know which pins i will manipulate directly later 

  PORTD |= PULL_HIGH; //pull the pins high, bitwise 'or' means that if the pin was originally low, it will be pulled to high (0|1 = 1)
  delayMicroseconds(1000); //0% throttle signal to initialize escs. 
  PORTD &= PULL_LOW;  //pull all the pins low, bitwise 'and' means that if the pin was originally high, 1&0 = 0, it will be pulled to low
  
  //==============done==============================
  Serial.begin(250000);
  //===========ACCELGYRO SETUP BEGINS===============   
    Wire.begin();
    TWBR = 12; //prescaler for 400KHz i2c clock rate, you may or may not use it, it wont really make a huge difference as the cycle time is fixed anyway
    
    accelgyro.initialize();  //do the whole initial setup thingy using this function.
    accelgyro.testConnection()==1 ? connection=1 : connection=0 ; 
    // offsets
   //2068.91||-1138.95||16216.07||-37.22||-13.16||-74.54||
   //3158.88||-428.89||16113.09||-27.64||-12.16||-72.98||
   
    offsetA[0]= 2068;        //these offsets were calculated beforehand
    offsetA[1]= -1138; 
    offsetA[2]= -143;
    offsetG[0]= -38;
    offsetG[1]= -13;
    offsetG[2]= -75;
    
    for(j=0;j<2000;j++)   //taking 2000 samples for finding initial orientation,takes about 0.8 seconds  
    {
      accelgyro.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);
      for(i=0;i<2;i++)                                      
      {
        A[i]=a[i];         //transfer value
        A[i]-=offsetA[i]; //subtracting offset
        A[i]*=0.0006103;    //convert to real world value
        lastA[i]+=A[i];    //store in lastA[i]
      }
    }
    A[0]=lastA[0]*0.0005;   //take average of 2000 readings
    A[1]=lastA[1]*0.0005;
    lastA[0]=0;  //lastA[i] was used here only as a place holder for a "sum" variable. it's purpose as a sum variable
    lastA[1]=0;  // has been fullfilled, therefore it will now be restored to 0 so that it can be used for it's origianl purpose
   
    T[0]=(+57.3*asin(A[1]*0.102));  //initial orientation 
    T[1]=(-57.3*asin(A[0]*0.102));
//------------ACCEL-GYRO SETUP ENDS-------------

}



int throttle=1000,dummy; //initializing throttle with default value
float rollsetp=0.0,yawsetp=0.0,pitchsetp=0.0,pError,rError; //initializing setpoints at 0
int p,r,y;    // initializing PID outputs for pitch, yaw, roll
long lastTime;  //timing variable for failsafe 
float tanSqTheta,cosSqTheta,cosTheta,Av=0,Vv=0,terror;
float sterror=0;
float dist,lastDist=0.2,lastV,delV=0;
volatile bool ultra=false;
int HOVERTHROTTLE=1500;
#define throttleKp 75
#define throttleKi 0.1875
#define throttleKd 5
#define dt 0.0025  //cycle time in seconds
#define my_sqrt(a) (0.25 + a*(1-(0.25*a))) // 30us, DO NOT USE IT FOR numbers too far away from 1, its an approximation for g's sake.

long failsafe=0;  //initializing failsafe variable 
volatile bool servoWrite=0; //initializing servoWrite variable as false(no signal received)
bool arm=0;  //initializing arming variable as false(initially not armed)
bool state=0; //initializing state as false so that the motor is not sent any signals
bool lowbatt = false;

void readMPU()   //function for reading MPU values. its about 80us faster than getMotion6() and hey every us counts!
{
  Wire.beginTransmission(0x68);  //begin transmission with the gyro
  Wire.write(0x3B); //start reading from high byte register for accel
  Wire.endTransmission();
  Wire.requestFrom(0x68,14); //request 14 bytes from mpu
  //280us for all data to be received. 
  //requrestFrom is a blocking function 

  //squeeze the vertical velocity calculations here.
  tanSqTheta = 0.0003045*(T[0]*T[0] + T[1]*T[1]); //35us
  cosSqTheta = 1 - tanSqTheta + (tanSqTheta*tanSqTheta); //20us
  cosTheta = my_sqrt(cosSqTheta);//30us
  Av = A[2]*cosTheta - 9.8*cosSqTheta; // 20us
  if(Av<1&&Av>-1) 
  {
    Av = 0;
  }
  if(ultra)
  {
    ultra = false;
    dist = input[5]*0.0002105*cosTheta;//caliberation number to convert to meters.
    Vv = (dist - lastDist)*10;//10Hz;
    delV = Vv - lastV;
    if((delV*delV*10)>(Av*Av))//30us
    {
      Vv = lastV;
    }
    lastDist = dist; 
    lastV = Vv;
  }//188us,31 to spare!

  esc_timer=micros();   //get time stamp
  PORTD |= PULL_HIGH;   //pull the pins high 
                        //notice that the pins are pulled high after getting the time stamp.this is done in that order because
                        //the micros() function returns the time at which it was called and not at which it returns, so there is a 3.5us
                        //delay. hence the time that we get in the esc_timer is 3.5us old.
                        //In the motorWrite function, the time stamp is taken first(which returns a 3.5 us old time) and then compared.
                        //This essentially reduces the error that can exist in the pulse's width because the order in which start time is 
                        //observed and pin is pulled high is the same as the order in which end time is observed and the pin is pulled low
  //each value in the mpu is stored in a "broken" form in 2 consecutive registers.(for example, acceleration along X axis has a high byte at 0x3B and low byte at 0x3C 
  //to get the actual value, all you have to do is shift the highbyte by 8 bits and bitwise add it to the low byte and you have your original value/. 
  a[0]=Wire.read()<<8|Wire.read();  
  a[1]=Wire.read()<<8|Wire.read(); 
  a[2]=Wire.read()<<8|Wire.read(); 
  g[0]=Wire.read()<<8|Wire.read();  //this one is actually temperature but i dont need temp so why waste memory.
  g[0]=Wire.read()<<8|Wire.read();  
  g[1]=Wire.read()<<8|Wire.read();
  g[2]=Wire.read()<<8|Wire.read();
}// 4 + 28(0.125)=11us . change this->in total it takes 291us. 11 us since esc_timer. 

int deadBand(int input)//the receiever signals vary a little bit (set value +/- 8us). This function removes that jitter 
{
  if(input>=1496&&input<=1504)  //if i m trying to send 1500 and the value received is between these 2, make it 1500 
  {
    return 1500;
  }
  return input;
}

float ExpKp(float k,float error)
{
 return k*(1 + CONSTANT*error*error);
}//35us function

void motorWrite() //has a maximum error of 3.5 us(time taken by micros() to return ) 
{        
   while(PORTD>=8)  //while any of the pins 3,4,5,6 are high
   {
     end_timer=micros();//store current time in a variable, better than calling micros() in the if condition, 
                        //for example if all the escs need to be fed 1500us, then there will be a delay of 3.5us(micros() takes 3.5us to return the value of time) 
                        //between each pulse's falling edge, meaning a difference of 14us between the first and the last motor.  
     if(end_timer >= FL)  
     {
        PORTD &= FLpin;
     }
     if(end_timer >= FR)
     {
        PORTD &= FRpin;
     }
     if(end_timer >= BL)
     {
        PORTD &= BLpin;
     }
     if(end_timer >= BR)
     {
        PORTD &= BRpin;
     }
   }
}

inline int limiter(int input)
{
  if(input>maxValue)
  {
    return maxValue;
  }
  if(input<minValue)
  {
    return minValue;
  }
  return input;
}//~4us

float dead_Zone(float input)
{
  if(input<3)
  {
    return 0;
  }
  if(input>-3)
  {
    return 0;
  }
  return input;
}//~4us

void correction()
{
   if(throttle>1800)
   {
      throttle=1800;    //cap the max throttle value
   }
   if(throttle>minValue)   //if throttle is above minimum value 
   {
     FL = limiter(throttle+p+r+y) + esc_timer;// adding esc_timer to FL  
     FR = limiter(throttle+p-r-y) + esc_timer;//saves us time in the if()
     BL = limiter(throttle-p+r-y) + esc_timer;//conditions in the motorWrite()
     BR = limiter(throttle-p-r+y) + esc_timer; //function,making it more precise 
                                               //and consuming less time as 
                                               //addition takes~4-5us
   }//23us .
   else
   {
      FL=(1000)+esc_timer;
      FR=(1000)+esc_timer;
      BL=(1000)+esc_timer;
      BR=(1000)+esc_timer;
   }//20us by now
   //843us since esc_timer . 157us buffer.

   motorWrite();  
} 

float hovercap(float input)
{
  if(input>150)
  {
    return 150;
  }
  if(input<-150)
  {
    return -150;
  }
  return input;
}//~4us

void loop()
{
 if(connection==0)  //in case accelgyro connection fails
 {
    lastTime = esc_timer = micros();     //get time stamp
    PORTD |= PULL_HIGH; //pull the pins high
    FL=(1000);    //send 0% throttle to all motors 
    FR=(1000);
    BL=(1000);
    BR=(1000);
    motorWrite();  //write the pwm values 
 }
 else
 {
   lastTime = micros();
   callimu();   //494us since esc_timer,this function calculates orientation (pitch and roll) 
  
   if(yawsetp<(-150)&&throttle<=minValue)   //arming sequence
   {
      arm=1;
   }
   else if(yawsetp>150&&throttle<=minValue)  //disarming sequence 
   {
      arm=0;
   }
   //4us ,498us since esc_timer
   
   //PID begins---
   pError = pitchsetp-T[0]; //storing the error value somewhere as it will be
   rError = rollsetp-T[1]; //used repeatedly
   sigma[0]+= (pError); //incrementing integral of error 
   sigma[1]+= (rError);
   //20us, 518us since esc_timer
   for(i=0;i<2;i++)
   {
      if(sigma[i]>IMAX)
      {
        sigma[i]=IMAX;  //capping max value of integral of error 
      }
      if(sigma[i]<(-IMAX))
      {
        sigma[i]=(-IMAX);
      }
   }//~4us, 522us since esc_timer   
   //PID (funny how colleges spend 1 month trying to explain something that can be written in a single line of code) 
   Kp_pitch = ExpKp(BaseKp,pError);
   Kp_roll = ExpKp(BaseKp,rError);
    
   r = Kp_roll*(rError) - BaseKd*G[1] + Ki*sigma[1];   //reducing time by not creating a function at all for these tiny tasks
   
   p = Kp_pitch*(pError) - BaseKd*G[0] + Ki*sigma[0];
   
   y = YawKp*(yawsetp-G[2]);
   if(y>0&&y>YAW_MAX) //capping max yaw value
   {
    y=YAW_MAX;
   }
   if(y<0&&y<-YAW_MAX)
   {
    y= -YAW_MAX;
   }
   //170us ,692us since esc_timer
    
   if(servoWrite) //when new pwm signals from receiver are received (see ISR function at the bottom) 
   {
      //transferring inputs from volatile to non-volatile variables 
      throttle = dummy =input[1];
      yawsetp  =(1500-deadBand(input[3]))*0.6;   //this is yaw rate(deg/sec)
      pitchsetp =(1500-deadBand(input[2]))*0.1;   //roll, pitch setp in degrees
      rollsetp=(deadBand(input[0])-1500)*0.1;   
      tune = input[4];
     
      servoWrite=0;     //making servo write false 
      failsafe=millis(); //giving time-stamp to the failsafe variable. failsafe is updated everytime a signal from the receiver is received.  
   }
   
   // 50us, 742us since esc_timer
   ((millis()-failsafe<1000)&&arm)? state=1 : state=0 ; //if receiver is still connected or reconnects within a second and the drone is "armed", state = 1  
   //5us, 748us since esc_timer

   if(lowbatt||((millis()-failsafe>1000)&&arm))//(if connection has been lost but we are mid air)
   {
      tune=2000;//forcefully go into alti-hold mode,
      state = 1;//forcefully make the state =1
      dummy = 1400;//drop with 20% full speed, should be about 40cm/s
      if(!lowbatt)
      {
        rollsetp = 0;
        pitchsetp = 0;
        yawsetp = 0; // ensure that it stays relatively flat 
      }
      if(dist<0.3)
      {
        state=0;  
      }
   }

   if(tune>1500)
   {
      terror = 0.01*(deadBand(dummy)-1500) - Vv;
      sterror += terror;
      throttle = hovercap(throttleKp*terror) + hovercap(throttleKi*sterror) -hovercap(throttleKd*Av) +  1400;//open loop + closed loop = advanced PID.
      if(input[1]<minValue)
      {
        throttle = minValue;
      }
   }//70us.820us since esc_timer
   
   //
   if(state)   
   {
      if(analogRead(A1)<600)//cell voltage below 3.5V under load
      {
        lowbatt = true;
      }
      correction(); //call the correction function to calculate the pwms and send the pwms to the escs    
   }
   else if(!state)  //if receiver does not reconnect within a second or is lost or the drone is dis-armed, kill throttle 
   {
      throttle=1000;
      correction();
      sigma[0]=0;  //reset integral errors to 0
      sigma[1]=0;
      T[0]=(+57.3*asin(A[1]*0.102));  //initial orientation 
      T[1]=(-57.3*asin(A[0]*0.102));
      
      Vv = 0;
      sterror = 0;
   }  
   
 }
 //Serial.println(Vv);
 //Serial.print(byte(r));
 while(micros()-lastTime<2500);  //wait for the 2500 us to be over
}



ISR(PCINT0_vect)
{
  timer[0]=micros();
  //channel 1 ----
  
  if(last_channel[0]==0&& PINB & B00000001) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[0]=1;
    timer[1]=timer[0];          
  }
  else if(last_channel[0]==1 && !(PINB & B00000001))
  {
    last_channel[0]=0;
    input[0]=timer[0]-timer[1];
  }

  //channel 2---
  if(last_channel[1]==0&& PINB & B00000010) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[1]=1;
    timer[2]=timer[0];          
  }
  else if(last_channel[1]==1 && !(PINB & B00000010))
  {
    last_channel[1]=0;
    input[1]=timer[0]-timer[2];
  }

  //channel 3---
  if(last_channel[2]==0&& PINB & B00000100) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[2]=1;
    timer[3]=timer[0];          
  }
  else if(last_channel[2]==1 && !(PINB & B00000100))
  {
    last_channel[2]=0;
    input[2]=timer[0]-timer[3];
  }
  
  //channel 4---
  if(last_channel[3]==0&& PINB & B00001000) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[3]=1;
    timer[4]=timer[0];          
  }
  else if(last_channel[3]==1 && !(PINB & B00001000))
  {
    last_channel[3]=0;
    input[3]=timer[0]-timer[4];
    
    servoWrite=true;  //servo write becomes true when the last signal comes in
  }

  if(last_channel[4]==0&& PINB & B00010000) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[4]=1;
    timer[5]=timer[0];          
  }
  else if(last_channel[4]==1 && !(PINB & B00010000))
  {
    last_channel[4]=0;
    input[4]=timer[0]-timer[5];
  }
}

ISR(PCINT1_vect,ISR_NOBLOCK)
{
  timer[7]=micros();
  //channel 1 ----
  
  if(last_channel[5]==0&& PINC & B00000001) //makes sure that the first pin was initially low and is now high
  {                                         //PINC & B00000001 is equivalent to digitalRead but faster
    last_channel[5]=1;
    timer[6]=timer[7];           
  }
  else if(last_channel[5]==1 && !(PINC & B00000001))
  {
    last_channel[5]=0;
    input[5]=timer[7]-timer[6];
    ultra = true;
  }
}


