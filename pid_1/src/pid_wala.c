#include "vrep.h"

// for more info refer to 
// https://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm 

//C Headers
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

#define MAX_PWM 5
#define MIN_PWM 1

//Line Following Tuning Parameters
float yaw_kP= 0.56;
float yaw_kI= 0.1;
float yaw_kD= 0.17;

//FOR LINE FOLLOWING
float yaw_error=0, yaw_prev_error=0, yaw_difference=0, yaw_cumulative_error=0, yaw_correction=0;
int weights[4] = {3,1,-1,-3};

float left_pwm = 0, right_pwm = 0;


float constrain(float x, float lower_limit, float higher_limit);
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float left_pwm,float right_pwm);

int main(int argc,char* argv[])
{

    // all handles that will be passed from lua to this code
    int portNb=-1;
    int leftMotorHandle;
    int rightMotorHandle;  
    int botbase;
	int reference;
	int sensor[4];
    // number of variables passed from lua to this code + 1
    // (+1 because the first value in argv is always the location of this program)
    int n = 8;

    // save the handles
    if (argc>=n){
        portNb=atoi(argv[1]); 
        leftMotorHandle = atoi(argv[2]);
        rightMotorHandle = atoi(argv[3]);
		sensor[0] = atoi(argv[4]);
		sensor[1] = atoi(argv[5]);
		sensor[2] = atoi(argv[6]);
		sensor[3] = atoi(argv[7]);
    }
    else{
        printf("Sufficient arguments not provided'!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // variables for the code
    float angle[3] = {0,0,0};
   // float leftMotorSpeed = 1;
    //float rightMotorSpeed = 1;
    float* auxValues = NULL;
    int* auxValuesCount = NULL;
	char s;
	int c=0;
	float svalue[4];
    // for details on these functions refer: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm

    // start communication with coppeliasim
    int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);

    if (clientID!=-1){
        printf("connection established...\n");
		float time=0;
        /******************** SETUP ******************************/
        // write code here that you wish to perform ONCE
		//printf("motion type:\n");
		//scanf("%c",&s);
		while (simxGetConnectionId(clientID)!=-1){
                		
            //reading sensors
			simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               svalue[0] = auxValues[10];
            }
			simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               svalue[1] = auxValues[10];
            }
			simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               svalue[2] = auxValues[10];
            }
			simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               svalue[3] = auxValues[10];
            }
            printf("the sensor values: %f %f %f %f\n",svalue[0],svalue[1],svalue[2],svalue[3]);


          	//calculating_yaw_error :-
			int all_black_flag = 1;
    		float weighted_sum = 0, sum = 0, pos = 0;
    
    		for(int i = 0; i < 4; i++){
        		if(svalue[i] > 0.6)
        			all_black_flag = 0;
       			weighted_sum += (float)(svalue[i]) * (weights[i]);
        		sum += svalue[i];
      		}
    		if(sum != 0)
    			pos = weighted_sum / sum;

			if(all_black_flag == 1)
    		{
        		if(yaw_error > 0)
            		pos = 0.2;
        		else
            		pos = -0.2;
    		}

    		yaw_error = pos;
        	//calculating_yaw_correction :-
			//yaw_error *= 10;
    		yaw_difference = (yaw_error - yaw_prev_error);
    		yaw_cumulative_error += yaw_error;
    
    		if(yaw_cumulative_error > 3)
    		    yaw_cumulative_error = 3;
        
    		else if(yaw_cumulative_error < -3)
    		    yaw_cumulative_error = -3;
    
			yaw_correction = yaw_kP*yaw_error + yaw_kI*yaw_cumulative_error + yaw_kD*yaw_difference;
		    yaw_prev_error = yaw_error;
           	right_pwm = constrain(( yaw_correction), MIN_PWM, MAX_PWM);
            left_pwm = constrain((-1*yaw_correction), MIN_PWM, MAX_PWM);
                       
            //Extra yaw correction during turns
            if(yaw_error>0.1)
            {
                right_pwm+=1.5;
                left_pwm-=1.5;   
            }
            else if(yaw_error<-0.1)
            {
                left_pwm+=1.5;
                right_pwm-=1.5;
            }
                
         	move(clientID,leftMotorHandle,rightMotorHandle,left_pwm,right_pwm);

        	extApi_sleepMs(50);
        	        /*********************************************************/
        }
        
        // end communication with lua
        simxFinish(clientID);

    }
    printf("Simulation Stopped!");
}
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float left_pwm,float right_pwm)
{
	simxSetJointTargetVelocity(clientID,leftMotorHandle,left_pwm,simx_opmode_oneshot);
	simxSetJointTargetVelocity(clientID,rightMotorHandle,right_pwm,simx_opmode_oneshot);
}
float constrain(float x, float lower_limit, float higher_limit)
{
    if(x < lower_limit)
        x = lower_limit;
    
    else if(x > higher_limit)
        x = higher_limit;

    return x;
}

