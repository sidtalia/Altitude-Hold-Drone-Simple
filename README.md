# Altitude-Hold-Drone-Simple

video link : https://www.youtube.com/watch?v=D5vC0Eu9LyE 

Has 3 modes : 

1)Rate mode : when aux channel is at the lowest position(pwm less than 1200us).

2)Angle mode : when aux channel is above 1400us.

3)Altitude hold mode : when aux channel is above 1800us. 

Turning the thing ON: 

1)Connect the battery. wait for the esc's to complete their beeping process.

2)Once they're done beeping, hit the reset button on the arduino.

ARMING-DISARMING SEQUENCE: 

1)Keep throttle at lowest position(less than 1100us pulse on throttle channel)

2)ARMING : push the left stick (yaw and throttle) to the right (yaw channel pulse more than 1750us)

3)DISARMING : push the left stick to the left (yaw channel pulse less than 1250us)


If you change any of the following you might have to find your own PID settings

Preferred thing : name of thing (reason for thing) 

1)Preferred frame : dji 450 (readily available)

2)Preferred TxRx : flysky fs-i6. (this is important for the receiver disconnect failsafe to work).

3)Preferred battery : 2200mAh 3s (orange batteries, 40C) (a lot of current needed for quick movements)

4)Preferred esc : simonK esc's (3s) (great throttle response. can keep up with really fast inputs) 

5)Preferred motors : 1000Kv (efficiency and speed both)

6)Preferred propellers : 1038 (10x3.8) ABS propeller (smooth, less vibration, less noise and the best part: ABS doesn't crack, it just   
  bends a little when you hit the ground and you can just un-bend it :P). (for god's sake do not use the carbon fiber dji copy 1038        propellers they're too heavy and you'll have the worst experience trying to fly with them with 1000kv motors. Use 900kv or lower        motors for them)

7)Preferred ultrasonic sensor : maxbotix mb1242 (the ones that communicate via pwm). place the ultrasonic sensor facing downwards at the 
                              center of the bottom plate.


HARDWARE SETUP: use a 5V 16MHz mini pro(or an uno, but i like the mini pro cuz it's small).

Front left esc  -> pin 3

Front right esc -> pin 4

Back left esc   -> pin 5

Back right esc  -> pin 6

Roll channel    -> pin 8

Throttle channel-> pin 9

Pitch channel   -> pin 10

Yaw channel     -> pin 11 

Aux channel     -> pin 12

Ultrasonic sensor ->pin A0

A4 (SDA) -> SDA of mpu6050

A5 (SCL) -> SCL of mpu6050

Battery's 1st cell ->pin A1


SOFTWARE SETUP: 

1)Clone this repo put it in your folder of arduino sketches.

2)Clone my mpu-library(which is a copy of the jrowberg library with slight changes) repo and put that in the libraries folder that you     have inside the arduino folder.

3)For the mpu offsets, run the mpu offset sketch. first 3 values are offsets for accel Ax,Ay,Az(offsetA[0], offsetA[1], 
  offsetA[2]. subtract offsetA[2] from 16072.7 for the actual offsetA[2] (for example, check out the offsetA[2] in my sketch. your         offset should something like that)). The next 3 values are offsets for gyro Gx, Gy, Gz(offsetG[0], offsetG[1], offsetG[2]).


TESTING THE THING FOR THE FIRST TIME: 

1)Remove all propellers

2)Make sure you have the offsets

3)Make sure the esc's are caliberated with 2000us max pulse and 1000us min pulse

4)Connect the battery monitor pin(pin A1) to 5V to bypass the battery failsafe entirely.

5)ARM the quadcopter, switch to angle hold mode. the motors should try moving a little bit(may not move continuosly)

6)Switch to alti-hold mode, the motors should start moving continuosly even if you have not armed the quad.

7)DISARM the quadcopter after taking it back into angle mode. The motors should stop. 

8)Mount the propellers (front left->clock wise, front right->counter clock wise, back left->counter clock wise, back right->clock wise)

9)Make sure the propellers are mounted tightly.

10)ARM the quadcopter and put it in angle mode. Hold the quadcopter from underneath. Give 10% throttle (kill throttle if anything goes      wrong). Give small pitch and roll inputs. the max pitch angle is +/-50degrees. 10% of that would be like 5 degrees. So check to see      if it corresponds to the small inputs. Do not give inputs larger than 10% while it is still in your hand. 

11)Make sure it is not vibrating un-controllably (ideally it wont because I have tested these PID values over a hundred flights and     
   haven't had any problems yet).

12)When flying for the first time(if you've never flown a quad before), keep the quad at a height less than one meter. Preferably learn    to fly with a toy quad copter(the really small ones) before flying this.

13)If the response is too fast for you, turn down the CONSTANT value in the main sketch to half it's original value and keep halfing it    till you get desired response.





