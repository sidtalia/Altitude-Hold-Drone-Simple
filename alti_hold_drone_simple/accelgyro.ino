// every function is inline because fuck you thats why. 
/*
╔═══════╗╔═╗─╔═╗╔═╗─╔═╗╔══════╗
║███████║║█║─║█║║█║─║█║║██████║
║█╔╗█╔╗█║║█║─║█║║█║─║█║║█╔══╗█║
╚═╝║█║╚═╝║█╚═╝█║║█║─║█║║█║──╚═╝
───║█║───║█████║║█║─║█║║█║
───║█║───║█╔═╗█║║█║─║█║║█║╔═══╗
───║█║───║█║─║█║║█╚═╝█║║█╚╝█▀█║
───║█║───║█║─║█║║█████║║█▄▄▄▄█║
───╚═╝───╚═╝─╚═╝╚═════╝╚══════╝
╔═╗────╔═══╗╔════╗╔════╗
║█║────║███║║████║║████║
║█║────╚╣█╠╝║█╔══╝║█╔══╝
║█║─────║█║─║█╚══╗║█╚══╗
║█║─────║█║─║████║║████║
║█║─╔═╗─║█║─║█╔══╝║█╔══╝
║█╚═╝█║╔╣█╠╗║█║───║█╚══╗
║█████║║███║║█║───║████║
╚═════╝╚═══╝╚═╝───╚════╝ 
 */

inline __attribute__((always_inline)) float my_asin(float a)
{
  return a*(1+(0.5*a*a)); //35us
}

inline void callimu()
{
  //-------EXTRACTION AND PROCESSING OF ACCEL-GYRO DATA BEGINS--
  readMPU();  //this function is marginally faster for the purpose of reading. i use the library functions in the setup because convenience. 
                //this is the only function which is being called repeatedly so this is the only function worth redefining .
  if(a[0]==0&&a[1]==0&&a[2]==0&&g[0]==0&&g[1]==0&&g[2]==0)//this is in case the accelgyro dicks me over and goes dead, I need some way of preventing the quad from
  {                                                           //going out of control 
    G[0] += 0.01055*p*dt; //i found the number "0.01055" by solving a bunch of math and physics. its the transfer function constant if you've studied control systems. 
    G[1] += 0.01055*r*dt;
    G[2] = 0;
    T[0] += G[0]*dt;
    T[1] += G[1]*dt;
    accelgyro.initialize();//keep trying to re-initialize the mpu
  }
  else
  {
    for(i=0;i<3;i++)
    {
      A[i]=a[i];
      A[i]-=offsetA[i]; //subtracting offset
      A[i]*=0.0006103; //mapping to real world values in m/s^2 , instead of dividing by 1638.4 i multiply by 1/1638.4 to save on time as multiplication is faster 
      
      A[i]= (0.5*A[i])+(0.5*lastA[i]); //applying the steady state assumption that the Acceleration can't change too much within 2.5 ms 
      lastA[i]=A[i]; 
  
      G[i]=g[i];
      G[i]-=offsetG[i]; // subtracting offset
      G[i]*=0.030516;   //mapping to degrees per second for FS_1000
      G[i]=(0.7*G[i])+(0.3*lastG[i]);  //buffer filter,same as that for accel.
      lastG[i]=G[i]; 
    }   //243us in this function. 11+243 = 254us since esc_timer 
    //-----EXTRACTION OF ACCEL-GYRO DATA ENDS-----
    orientationUpdate();   //~240us function   
  }
}                            //494us since esc_timer.


inline void orientationUpdate()    
{
  
  T[0]+=float(G[0]*dt);  //T[0]=pitch,T[1]=roll,T[2]=yaw.    //~30us 
  T[1]+=float(G[1]*dt);
  //yaw compensation for pitch and roll
  /*actual formula- 
   * T[0] += T[1]*sin(G[2]*0.0000436);
   * T[1] -= T[0]*sin(G[2]*0.0000436);
   * even this is an approximation from sin(pitch)=sin(roll)*sin(yaw) as the pitch vs yaw curve is approximately a sin function. 
  */
  T[0] += T[1]*(G[2]*0.0000436); //0.0000436=dt/57.3 because sin takes radian not degrees,also, sin(x)=x as x->0 .this saves me fucking 450us
  T[1] -= T[0]*(G[2]*0.0000436);  //because sin() takes fucking 255 us(max, min=180us) to execute. fuck that i can live with approximations
                                          // ~50us
  if((A[1]*A[1])<16&&(A[0]*A[0])<16)  //making sure that magnitude of A[0] and A[1] is less than 4.not using sqrt() because it takes 70us to execute once and i need to do it twice.               
  {
    //complimentary filter for accel-gyro fusion
    T[0]= 0.99*T[0]+0.573*my_asin(A[1]*0.102); // the accel gives the sign of the external force on an accelerating body, so if you are holding it flat, it is measuring force applied by you in the upward direction and not the earth's gravity
    T[1]= 0.99*T[1]-0.573*my_asin(A[0]*0.102); 
  } //~140us
}

