
//***************Libraries***************************
//all 6 piston pid control code
#include <PID_v1.h>
//using the power and root functions 
#include<math.h>
//**************PID_configurations********************
#define relay_open_1 2 //enable pin to open
#define relay_open_2 3 //enable pin to open
#define relay_open_3 4 //enable pin to open
#define relay_open_4 5 //enable pin to open
#define relay_open_5 6 //enable pin to open
#define relay_open_6 7 //enable pin to open

#define relay_close_1 8//enable pin to close 
#define relay_close_2 9//enable pin to close 
#define relay_close_3 10//enable pin to close 
#define relay_close_4 11//enable pin to close 
#define relay_close_5 12//enable pin to close 
#define relay_close_6 13//enable pin to close 

#define sensor_1 A1
#define sensor_2 A2
#define sensor_3 A3
#define sensor_4 A4
#define sensor_5 A5
#define sensor_6 A6

#define IN1_pid1 22//open valve HIGH
#define IN3_pid1 24//close Valve HIGH
#define IN1_pid2 26//open valve HIGH
#define IN3_pid2 28//close Valve HIGH
#define IN1_pid3 30//open valve HIGH
#define IN3_pid3 32//close Valve HIGH
#define IN1_pid4 34//open valve HIGH
#define IN3_pid4 36//close Valve HIGH
#define IN1_pid5 38//open valve HIGH
#define IN3_pid5 40//close Valve HIGH
#define IN1_pid6 42//open valve HIGH
#define IN3_pid6 44//close Valve HIGH

//**************joystick_pins_configurations****
#define SW_pin 53 // digital pin connected to SW the 
#define X_pin A8  // analog pin connected to VRx
#define Y_pin A9  // analog pin connected to VRy

//**************PID_variables********************
//some variables to increse the precision of the sensor readings/
double sum_reading_1=0;
double avg_reading_1=0; 
double sum_reading_2=0;
double avg_reading_2=0; 
double sum_reading_3=0;
double avg_reading_3=0; 
double sum_reading_4=0;
double avg_reading_4=0; 
double sum_reading_5=0;
double avg_reading_5=0; 
double sum_reading_6=0;
double avg_reading_6=0; 



int val_sensor1=0;//sensor1 reading 
int val_sensor2=0;//sensor2 reading
int val_sensor3=0;//sensor3 reading
int val_sensor4=0;//sensor4 reading
int val_sensor5=0;//sensor5 reading
int val_sensor6=0;//sensor6 reading


//sensors zero error
int zero_error_sensor1=0;
int zero_error_sensor2=0;
int zero_error_sensor3=0;
int zero_error_sensor4=0;
int zero_error_sensor5=0;
int zero_error_sensor6=0;

double Kp_1=0.65,Ki_1=0.8,Kd_1=0.01;  // pid_1 parameters values
double Kp_2=0.65,Ki_2=0.8,Kd_2=0.01;  // pid_2 parameters values
double Kp_3=0.65,Ki_3=0.8,Kd_3=0.01;  // pid_3 parameters values
double Kp_4=0.65,Ki_4=0.8,Kd_4=0.01;  // pid_4 parameters values
double Kp_5=0.65,Ki_5=0.8,Kd_5=0.01;  // pid_5 parameters values
double Kp_6=0.65,Ki_6=0.8,Kd_6=0.01;  // pid_6 parameters values


double Setpoint_1, dis_1, opening_output_1;//variables to use in the pid_1 function
double Setpoint_2, dis_2, opening_output_2;//variables to use in the pid_2 function
double Setpoint_3, dis_3, opening_output_3;//variables to use in the pid_3 function
double Setpoint_4, dis_4, opening_output_4;//variables to use in the pid_4 function
double Setpoint_5, dis_5, opening_output_5;//variables to use in the pid_5 function
double Setpoint_6, dis_6, opening_output_6;//variables to use in the pid_6 function



double commandMin_1 = 0;//lowest pwm pid_1 
double commandMax_1 = 255;//highest pwm pid_1
double commandMin_2 = 0;//lowest pwm pid_2 
double commandMax_2 = 255;//highest pwm pid_2
double commandMin_3 = 0;//lowest pwm pid_3 
double commandMax_3 = 255;//highest pwm pid_3
double commandMin_4 = 0;//lowest pwm pid_4 
double commandMax_4 = 255;//highest pwm pid_4
double commandMin_5 = 0;//lowest pwm pid_5 
double commandMax_5 = 255;//highest pwm pid_5
double commandMin_6 = 0;//lowest pwm pid_6 
double commandMax_6 = 255;//highest pwm pid_6



PID pid_1(&dis_1,&opening_output_1,&Setpoint_1,Kp_1,Ki_1,Kd_1,REVERSE);
PID pid_2(&dis_2,&opening_output_2,&Setpoint_2,Kp_2,Ki_2,Kd_2,REVERSE);
PID pid_3(&dis_3,&opening_output_3,&Setpoint_3,Kp_3,Ki_3,Kd_3,REVERSE);
PID pid_4(&dis_4,&opening_output_4,&Setpoint_4,Kp_4,Ki_4,Kd_4,REVERSE);
PID pid_5(&dis_5,&opening_output_5,&Setpoint_5,Kp_5,Ki_5,Kd_5,REVERSE);
PID pid_6(&dis_6,&opening_output_6,&Setpoint_6,Kp_6,Ki_6,Kd_6,REVERSE);


//**************stewart_platform_equation_variables********************

int a[6][6]={{333,376,42,-42,-376,-333},{242,168,-410,-410,168,242},{0,0,0,0,0,0},{42,739,696,-696,-739,-42},{828,-377,-451,-451,-377,828},{0,0,0,0,0,0}};
int R=1;//rotational matrix at zero angle
int Result[3];//array to store the summation of platform center and the coordination of joint j
int L[3];//array to store the diffrence between the platform coordination and the piston coodrination of the same joint
int Length[6];//lengths of joints
int value =0;// signal of the joystick
int stepx=0;// step in x axis
int stepy=0;// step in y axis
int x=0;// signal of x axis
int y=0;// signal for y axis
int z=1;// reset to the zero postion
int center[]= {0,0,660};// center of platform
int Joystick_center_offset[]={0,0,0};// step taken in each axis in movement
int Pi[]={0,0,0} // array to store the productof rotational matrix and  the coordination of platform of  joint j 
int thx; //yaw
int thy; //roll
int thz; //pitch

//****************functions********************
// Function Description:
// Calculation of length of joints 
// by inverse kinematics of stewart platform
// Input: step coming from joystick
// Output: six lengths of the joints
int calculations (int arr[3])
{
   //updating the center of the platform by the step coming from the joystick
   for (int i=0;i<3;i++){ 
     center[i]+=joystick_center_offset[i]; 
   }
   // Nested for loop to calculate the six lengths of the joints
// The variables used in this equation are vectors so in order
// to calculate them, we use a nested for loop that goes over each element
 for (int j=0;j<6;j++)
 {
      R=cos(thx)^2*cos(thy)^2*cos(thz)^2 + cos(thx)^2*cos(thy)^2*sin(thz)^2 + cos(thx)^2*cos(thz)^2*sin(thy)^2 + cos(thx)^2*sin(thy)^2*sin(thz)^2 + cos(thy)^2*cos(thz)^2/*   
      */*sin(thx)^2 + cos(thy)^2*sin(thx)^2*sin(thz)^2 + cos(thz)^2*sin(thx)^2*sin(thy)^2 + sin(thx)^2*sin(thy)^2*sin(thz)^2
   for(int i=0;i<3;i++)
   {
    // rotational matrix multiplied by the platform coordinates of joint j 
    Pi[i]=R*a[i][j];
   }

  for(int i=0;i<3;i++)
   {
      // summation of the center of the platform and the joints
    Result[i]=center[i]+Pi[i];
   }
for(int i=0;i<3;i++){
     // difference between the coordinates of the joint at platform and base
L[i]=Result[i]-a[i+3][j];
}
 // in order to get the length, we use dot product and remove the length 
 //of the fixed parts to get the extraction of the piston
Length[j]=(sqrt(pow(L[0],2)+pow(L[1],2)+pow(L[2],2)))-892;

if (Length[j]>=400)
{
    // this condition was added for safety as the maximum extraction is 400mm 
  Length[j]=400;
 
}
else if (Length[j]<=0)
{
  // this condition was added for safety so the piston doesn't break
  Length[j]=0;
}
Serial.println(Length[j]);  
}
}

//Function Description:
//This function get the signal from the joystick
//to get the Step signal for the platform
void signals ()
{
value = analogRead(0); //get the signal from the joystick
x=value-509;// calibration for the x-axis
if (x>10){stepx=1;}//sensor calibration
else if (x<-10){stepx=-1;}
else {stepx=0;}
 Serial.print("X:"); 
 Serial.print( stepx , DEC); 

 value = analogRead(1); //get the signal from the joystick
 y = value-497;// calibration for the y-axis
 if (y>10){stepy=1;}//sensor calibration
else if (y<-10){stepy=-1;}
else {stepx=0;}
 Serial.print(" | Y:"); 
 Serial.print(stepy, DEC); 

value = digitalRead(7);//get the signal from the joystick
z= value;
 Serial.print(" | Z: "); 
 Serial.println(z, DEC); 
delay(500);
     if(z==0) {//reset button
      Joystick_center_offset[0]=0;
      Joystick_center_offset[1]=0;
      Joystick_center_offset[2]=0;
      center[0]=0;
      center[1]=0;
      center[2]=670;
     }
   else if (((x>-3)&&(3>x)) && ((y>-3)&&(3>y)))
     {
      //for calibration
     }
     else if (x>0 && ((y>-3)&&(3>y))) 
     {//step in +ve direction in x axis
       Joystick_center_offset[0]=Joystick_center_offset[0]+30;
       Joystick_center_offset[2]=Joystick_center_offset[2]+30;
     }
     else if (x<0 && ((y>-3)&&(3>y)) )
     {//step in -ve direction in x axis
       Joystick_center_offset[0]=Joystick_center_offset[0]-30;
        Joystick_center_offset[2]=Joystick_center_offset[2]-15;
     }
     else if (y>0 && ((x>-3)&&(3>x))) 
     {//step in +ve direction in y axis
       Joystick_center_offset[1]=Joystick_center_offset[1]+30;
        Joystick_center_offset[2]=Joystick_center_offset[2]+30;
     }
     else if (y<0  && ((x>-3)&&(3>x)) )
     {//step in -ve direction in y axis
       Joystick_center_offset[1]=Joystick_center_offset[1]-30;
        Joystick_center_offset[2]=Joystick_center_offset[2]-15;
     }
     else if (x>0 && y>0)
     {//step in +ve direction in x & y axis
      Joystick_center_offset[0]=Joystick_center_offset[0]+30;
      Joystick_center_offset[1]=Joystick_center_offset[1]+30;
       Joystick_center_offset[2]=Joystick_center_offset[2]+15;
     }
     else if (x<0 && y<0)
     {//step in -ve direction in x & y axis
      Joystick_center_offset[0]=Joystick_center_offset[0]-30;
      Joystick_center_offset[1]=Joystick_center_offset[1]-30;
       Joystick_center_offset[2]=Joystick_center_offset[2]-15;
     }
     else if (x>0 && y<0)
     {//step in +ve direction in x axis and -ve y axix
     Joystick_center_offset[0]=Joystick_center_offset[0]+30;
      Joystick_center_offset[1]=Joystick_center_offset[1]-30;
       Joystick_center_offset[2]=Joystick_center_offset[2]-10;
     }
      else if (x<0 && y>0)
     {//step in -ve direction in x axis and +ve y axix
      Joystick_center_offset[0]=Joystick_center_offset[0]-30;
      Joystick_center_offset[1]=Joystick_center_offset[1]+30;
       Joystick_center_offset[2]=Joystick_center_offset[2]-10;
     }
}
//Function Description:
//comparing the error of the 6 pistons
//for safety and protection of the platform
//Input: errors coming from the sensors
//Output: stop movement if errors out of the range
int ratio (int error[6])
{
  for(int i=0;i<6;i++)
  {
   for(int j=1;j<6;j++)
    {
      if (error[i]/error[j]>3)
      break;
    }
  }
}

void setup() 
{
  //**********joystick_parameters_configuration************/
  pinMode(SW_pin, INPUT);
  //the switch logic is converted the low is high and the high is low
  digitalWrite(SW_pin, HIGH);//SW is off
  
  //**********pid_parameters_configuration************/
  Setpoint_1 =0; //postion desired for piston1
  Setpoint_2 =0; //postion desired for piston2
  Setpoint_3 =0; //postion desired for piston3
  Setpoint_4 =0; //postion desired for piston4
  Setpoint_5 =0; //postion desired for piston5
  Setpoint_6 =0; //postion desired for piston6

  //setting the pins configuration for both of the solenoid valves
  //and the wire potentiometer sensor signal
  //and the H-bridge IN pins  
   pinMode(relay_open_1, OUTPUT);
   pinMode(relay_close_1, OUTPUT);
   pinMode(relay_open_2, OUTPUT);
   pinMode(relay_close_2, OUTPUT);
   pinMode(relay_open_3, OUTPUT);
   pinMode(relay_close_3, OUTPUT);
   pinMode(relay_open_4, OUTPUT);
   pinMode(relay_close_4, OUTPUT);
   pinMode(relay_open_5, OUTPUT);
   pinMode(relay_close_5, OUTPUT);
   pinMode(relay_open_6, OUTPUT);
   pinMode(relay_close_6, OUTPUT);

    pinMode (sensor_1, INPUT);
    pinMode (sensor_2, INPUT);
    pinMode (sensor_3, INPUT);    
    pinMode (sensor_4, INPUT);
    pinMode (sensor_5, INPUT);
    pinMode (sensor_6, INPUT);

    pinMode (IN1_pid1, OUTPUT); 
    pinMode (IN3_pid1, OUTPUT);
    pinMode (IN1_pid2, OUTPUT); 
    pinMode (IN3_pid2, OUTPUT);
    pinMode (IN1_pid3, OUTPUT); 
    pinMode (IN3_pid3, OUTPUT);
    pinMode (IN1_pid4, OUTPUT); 
    pinMode (IN3_pid4, OUTPUT);
    pinMode (IN1_pid5, OUTPUT); 
    pinMode (IN3_pid5, OUTPUT);
    pinMode (IN1_pid6, OUTPUT); 
    pinMode (IN3_pid6, OUTPUT);


//setting the pids mode and limits 
    pid_1.SetMode(AUTOMATIC);
    pid_2.SetMode(AUTOMATIC);
    pid_3.SetMode(AUTOMATIC);
    pid_4.SetMode(AUTOMATIC);
    pid_5.SetMode(AUTOMATIC);
    pid_6.SetMode(AUTOMATIC);

    pid_1.SetOutputLimits(commandMin_1, commandMax_1);
    pid_2.SetOutputLimits(commandMin_2, commandMax_2);
    pid_3.SetOutputLimits(commandMin_3, commandMax_3);
    pid_4.SetOutputLimits(commandMin_4, commandMax_4);
    pid_5.SetOutputLimits(commandMin_5, commandMax_5);
    pid_6.SetOutputLimits(commandMin_6, commandMax_6);    

    Serial.begin(9600);
}

void loop() 
{

    while (x==0 && y==0 && z==1)
    {
       signals();
    }
   calculations(joystick_center_offset);
   x=0;
   y=0;
   z=1;
  }

  
  //the next 6 for loops are only a trick to eliminate some of the noise affecting the sensors readings
  for(int i=0;i<20;i++)
  {
   //this for loop only a trick to increase the precision of sensor1
   val_sensor1 = analogRead(sensor_1);//take reading from the sensor
   sum_reading_1=sum_reading_1+val_sensor1; 
  }
  avg_reading_1=sum_reading_1/20;
  dis_1=map(avg_reading_1,0,1023,0,500)-zero_error_sensor1;//mapping to get the real value - zero error1
  sum_reading_1=0; //clearing the sum

  for(int i=0;i<20;i++)
  {
   //this for loop only a trick to increase the precision of sensor2 
   val_sensor2 = analogRead(sensor_2);//take reading from the sensor
   sum_reading_2=sum_reading_2+val_sensor2; 
  } 
  avg_reading_2=sum_reading_2/20;
  dis_2=map(avg_reading_2,0,1023,0,500)-zero_error_sensor2;//mapping to get the real value - zero error2
  sum_reading_2=0; //clearing the sum

  for(int i=0;i<20;i++)
  {
   //this for loop only a trick to increase the precision of sensor3 
   val_sensor3 = analogRead(sensor_3);//take reading from the sensor
   sum_reading_3=sum_reading_3+val_sensor3; 
  } 
  avg_reading_3=sum_reading_3/20;
  dis_3=map(avg_reading_3,0,1023,0,500)-zero_error_sensor3;//mapping to get the real value - zero error3
  sum_reading_3=0; //clearing the sum

  for(int i=0;i<20;i++)
  {
   //this for loop only a trick to increase the precision of sensor4
   val_sensor4 = analogRead(sensor_4);//take reading from the sensor
   sum_reading_4=sum_reading_4+val_sensor4; 
  } 
  avg_reading_4=sum_reading_4/20;
  dis_4=map(avg_reading_4,0,1023,0,500)-zero_error_sensor4;//mapping to get the real value - zero error4
  sum_reading_4=0; //clearing the sum

  for(int i=0;i<20;i++)
  {
   //this for loop only a trick to increase the precision of sensor5 
   val_sensor5 = analogRead(sensor_5);//take reading from the sensor
   sum_reading_5=sum_reading_5+val_sensor5; 
  } 
  avg_reading_5=sum_reading_5/20;
  dis_5=map(avg_reading_5,0,1023,0,500)-zero_error_sensor5;//mapping to get the real value - zero error5
  sum_reading_5=0; //clearing the sum

  for(int i=0;i<20;i++)
  {
   //this for loop only a trick to increase the precision of sensor6
   val_sensor6 = analogRead(sensor_6);//take reading from the sensor
   sum_reading_6=sum_reading_6+val_sensor6; 
  } 
  avg_reading_6=sum_reading_6/20;
  dis_6=map(avg_reading_6,0,1023,0,500)-zero_error_sensor6;//mapping to get the real value - zero error6
  sum_reading_6=0; //clearing the sum

//after taking the 6 feedbacks from the sensors 
//calculate the pid response

pid_1.Compute();//the pid_1 calculating the error in each cycle for piston1
pid_2.Compute();//the pid_2 calculating the error in each cycle for piston2
pid_3.Compute();//the pid_3 calculating the error in each cycle for piston3
pid_4.Compute();//the pid_4 calculating the error in each cycle for piston4
pid_5.Compute();//the pid_5 calculating the error in each cycle for piston5
pid_6.Compute();//the pid_6 calculating the error in each cycle for piston6


//after calculating the six pid outputs 
//the next if conditions will allow the movment of the piston
//either opening or closing according to the error direction


//piston1 response 
  if((Setpoint_1-dis_1)>0)
    {
    pid_1.SetControllerDirection(DIRECT);//direct to open the piston/
    digitalWrite(IN1_pid1,1);/*setting the pin of open enable */
    digitalWrite(IN3_pid1,0);/*clearing the pin of close enbale  */
    analogWrite(relay_open_1, opening_output_1);//write the generated pwm on the open enable pin/
    }
    else if((Setpoint_1-dis_1)<0)
    {
    pid_1.SetControllerDirection(REVERSE);//reverse to close the piston/
    digitalWrite(IN1_pid1,0);/*clearing the pin of open enable */
    digitalWrite(IN3_pid1,1);/*setting the pin of close enbale */
    analogWrite(relay_close_1, opening_output_1);//write the generated pwm on the close enable pin/
    }
    else if ((Setpoint_1-dis_1)==0)
    {
    //if error equal zero stay at this postion/
    digitalWrite(IN1_pid1,0);
    digitalWrite(IN3_pid1,0); 
    }
    
//piston2 response
  if((Setpoint_2-dis_2)>0)
    {
    pid_2.SetControllerDirection(DIRECT);//direct to open the piston/
    digitalWrite(IN1_pid2,1);/*setting the pin of open enable */
    digitalWrite(IN3_pid2,0);/*clearing the pin of close enbale  */
    analogWrite(relay_open_2, opening_output_2);//write the generated pwm on the open enable pin/
    }
    else if((Setpoint_2-dis_2)<0)
    {
    pid_2.SetControllerDirection(REVERSE);//reverse to close the piston/
    digitalWrite(IN1_pid2,0);/*clearing the pin of open enable */
    digitalWrite(IN3_pid2,1);/*setting the pin of close enbale */
    analogWrite(relay_close_2, opening_output_2);//write the generated pwm on the close enable pin/
    }
    else if ((Setpoint_2-dis_2)==0)
    {
    //if error equal zero stay at this postion/
    digitalWrite(IN1_pid2,0);
    digitalWrite(IN3_pid2,0); 
    }

//piston3 response  
  if((Setpoint_3-dis_3)>0)
    {
    pid_3.SetControllerDirection(DIRECT);//direct to open the piston/
    digitalWrite(IN1_pid3,1);/*setting the pin of open enable */
    digitalWrite(IN3_pid3,0);/*clearing the pin of close enbale  */
    analogWrite(relay_open_3, opening_output_3);//write the generated pwm on the open enable pin/
    }
    else if((Setpoint_3-dis_3)<0)
    {
    pid_3.SetControllerDirection(REVERSE);//reverse to close the piston/
    digitalWrite(IN1_pid3,0);/*clearing the pin of open enable */
    digitalWrite(IN3_pid3,1);/*setting the pin of close enbale */
    analogWrite(relay_close_3, opening_output_3);//write the generated pwm on the close enable pin/
    }
    else if ((Setpoint_3-dis_3)==0)
    {
    //if error equal zero stay at this postion/
    digitalWrite(IN1_pid3,0);
    digitalWrite(IN3_pid3,0); 
    }

//piston4 response
  if((Setpoint_4-dis_4)>0)
    {
    pid_4.SetControllerDirection(DIRECT);//direct to open the piston/
    digitalWrite(IN1_pid4,1);/*setting the pin of open enable */
    digitalWrite(IN3_pid4,0);/*clearing the pin of close enbale  */
    analogWrite(relay_open_4, opening_output_4);//write the generated pwm on the open enable pin/
    }
    else if((Setpoint_4-dis_4)<0)
    {
    pid_4.SetControllerDirection(REVERSE);//reverse to close the piston/
    digitalWrite(IN1_pid4,0);/*clearing the pin of open enable */
    digitalWrite(IN3_pid4,1);/*setting the pin of close enbale */
    analogWrite(relay_close_4, opening_output_4);//write the generated pwm on the close enable pin/
    }
    else if ((Setpoint_4-dis_4)==0)
    {//if error equal zero stay at this postion/
    digitalWrite(IN1_pid4,0);
    digitalWrite(IN3_pid4,0); 
    }
    
//piston5 response
  if((Setpoint_5-dis_5)>0)
    {
    pid_5.SetControllerDirection(DIRECT);//direct to open the piston/
    digitalWrite(IN1_pid5,1);/*setting the pin of open enable */
    digitalWrite(IN3_pid5,0);/*clearing the pin of close enbale  */
    analogWrite(relay_open_5, opening_output_5);//write the generated pwm on the open enable pin/
    }
    else if((Setpoint_5-dis_5)<0)
    {
    pid_5.SetControllerDirection(REVERSE);//reverse to close the piston/
    digitalWrite(IN1_pid5,0);/*clearing the pin of open enable */
    digitalWrite(IN3_pid5,1);/*setting the pin of close enbale */
    analogWrite(relay_close_5, opening_output_5);//write the generated pwm on the close enable pin/
    }
    else if ((Setpoint_5-dis_5)==0)
    {
    //if error equal zero stay at this postion/
    digitalWrite(IN1_pid5,0);
    digitalWrite(IN3_pid5,0); 
    }

//piston6 response
  if((Setpoint_6-dis_6)>0)
    {
    pid_6.SetControllerDirection(DIRECT);//direct to open the piston/
    digitalWrite(IN1_pid6,1);/*setting the pin of open enable */
    digitalWrite(IN3_pid6,0);/*clearing the pin of close enbale  */
    analogWrite(relay_open_6, opening_output_6);//write the generated pwm on the open enable pin/
    }
    else if((Setpoint_6-dis_6)<0)
    {
    pid_6.SetControllerDirection(REVERSE);//reverse to close the piston/
    digitalWrite(IN1_pid6,0);/*clearing the pin of open enable */
    digitalWrite(IN3_pid6,1);/*setting the pin of close enbale */
    analogWrite(relay_close_6, opening_output_6);//write the generated pwm on the close enable pin/
    }
    else if ((Setpoint_6-dis_6)==0)
    {
    //if error equal zero stay at this postion/
    digitalWrite(IN1_pid6,0);
    digitalWrite(IN3_pid6,0); 
    }


//display output
  Serial.print(Setpoint_1);
  Serial.print(" ");
  Serial.println(dis_1);//printing the current postion

Serial.print(" ");

  Serial.print(Setpoint_2);
  Serial.print(" ");
  Serial.println(dis_2);//printing the current postion

Serial.print(" ");

  Serial.print(Setpoint_3);
  Serial.print(" ");
  Serial.println(dis_3);//printing the current postion

Serial.print(" ");

  Serial.print(Setpoint_4);
  Serial.print(" ");
  Serial.println(dis_4);//printing the current postion

Serial.print(" ");

  Serial.print(Setpoint_5);
  Serial.print(" ");
  Serial.println(dis_5);//printing the current postion

Serial.print(" ");

  Serial.print(Setpoint_6);
  Serial.print(" ");
  Serial.println(dis_6);//printing the current postion

}
