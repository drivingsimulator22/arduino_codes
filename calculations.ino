#include<math.h>
// coordinates of the piston at platfotm and base 
int a[6][6]={{333,376,42,-42,-376,-333},{242,168,-410,-410,168,242},{0,0,0,0,0,0},{42,739,696,-696,-739,-42},{828,-377,-451,-451,-377,828},{0,0,0,0,0,0}}; 
int R; // variable for rotational matrix 
int Pi[]={0,0,0} // array to store the productof rotational matrix and  the coordination of platform of  joint j 
int Result[3];//array to store the summation of platform center and the coordination of joint j
int L[3];//array to store the diffrence between the platform coordination and the piston coodrination of the same joint
int Length[6];//lengths of joints
int thx; //yaw
int thy; //roll
int thz; //pitch
int center[]= {0,0,670}; // center of platform

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


void setup() {
 Serial.begin(9600);

}


void loop() { 
}
