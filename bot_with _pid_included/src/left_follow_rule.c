#include "vrep.h"

// for more info refer to 
// https://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm 

//C Headers
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX_PWM 10
#define MIN_PWM 1

//Line Following Tuning Parameters
float yaw_kP= 0.5;
float yaw_kI= 0.0;
float yaw_kD= 0.4;

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
	float sensor_value[4];
	float turn_speed= 1;
    // for details on these functions refer: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm

    // start communication with coppeliasim
    int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	int speed = 0.1;
	int a=0;
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
               sensor_value[0] = auxValues[10];
            }
			simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               sensor_value[1] = auxValues[10];
            }
			simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               sensor_value[2] = auxValues[10];
            }
			simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               sensor_value[3] = auxValues[10];
            }
            printf("the sensor values: %f %f %f %f\n",sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3]);
     		
		if(sensor_value[0]>0 && sensor_value[1]>0 && sensor_value[2]>0 && sensor_value[3]>0){


            if(sensor_value[0]<0.4 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.7)			//pure left or straight left turn
			{           

                while(simxGetConnectionId(clientID)!=-1)
                {   //reading sensor values
                 	simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[0] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[1] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[2] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[3] = auxValues[10];
            		}
            		printf("the sensor values l/sl***: %f %f %f %f\n",sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3]);
                                    
					if(sensor_value[0]<0.4 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.7)
         				move(clientID,leftMotorHandle,rightMotorHandle,turn_speed,turn_speed);
                   	else
                   	{
                   		if(sensor_value[0]>0.7 && sensor_value[1]>0.7 && sensor_value[2]>0.7 && sensor_value[3]>0.7)
                   		{
                   			if(sensor_value[0]>0.7 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.7)
							{		//straight line
									printf("bye bye pure left\n");
									break;
							}
							else
                          		move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);    	//turns left
                   		}
           				else
           				{
                   			 	if(sensor_value[0]>0.7 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.7)
								{		//straight line
									a = a + 1;
									if( a == 2)
									{  	a = 0;
										printf("bye bye straight left\n");
										break;
									}
								}
								else
                          			move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);    	//turns left
                         }
                     }
            	}	
			}
			
					

		    else if(sensor_value[3]<0.4 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[0]>0.7)			//pure right or straight right turn detected
			{        
		
				while(simxGetConnectionId(clientID)!=-1)
                {   //reading sensors
					simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[0] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[1] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[2] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[3] = auxValues[10];
            		}
            		printf("the sensor values r/sr***: %f %f %f %f\n",sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3]);

					if(sensor_value[0]>0.7 && sensor_value[1]<0.7 && sensor_value[2]<0.7 && sensor_value[3]<0.7)
                    	move(clientID,leftMotorHandle,rightMotorHandle,turn_speed,turn_speed);
                   
                   else
                   { 	if(sensor_value[0]>0.7 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.7)
						{	//straight line
							printf("ByE bYe \n");
							break;
						}
						else
                    	   	move(clientID,leftMotorHandle,rightMotorHandle,turn_speed,-1*turn_speed);		//turns right
					}
				}
			}
			


			else if(sensor_value[3]<0.4 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[0]<0.4)								//for junction or t-shape
			{
				while(simxGetConnectionId(clientID)!=-1)
                {   //reading sensors
					simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[0] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[1] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[2] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[3] = auxValues[10];
            		}
            		printf("the sensor values j/t***: %f %f %f %f  \n",sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3]);

					if(sensor_value[0]<0.4 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]<0.4)
					{	printf("j/t khtm hone tak straight\n");
                    	move(clientID,leftMotorHandle,rightMotorHandle,turn_speed,turn_speed);
					}
					else
					{
						if(sensor_value[0]>0.4 && sensor_value[1]>0.4 && sensor_value[2]>0.4 && sensor_value[3]>0.4)		//t-shape	
						{	
						  	while(simxGetConnectionId(clientID)!=-1)
                		  	{   //reading sensors
								simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
								if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               						sensor_value[0] = auxValues[10];
            					}
								simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
								if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               						sensor_value[1] = auxValues[10];
            					}
								simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
								if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               						sensor_value[2] = auxValues[10];
            					}
								simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
								if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               						sensor_value[3] = auxValues[10];
            					}
            					printf("the sensor values t-shape***: %f %f %f %f  \n",sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3]);
								if(sensor_value[0]>0.7 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.7)			
								{	//straight line
										c = 1;
										printf("ByE bYe t-shape\n");
										break;
								
								}
								else{	printf("t-shape aaya so left turn\n");
                    			   	move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns left
								}
							 }
						  }
						  else if(sensor_value[0]>0.4 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.4)	      //for junction
						  {
							while(simxGetConnectionId(clientID)!=-1)
                		  	{   //reading sensors
								simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
								if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               						sensor_value[0] = auxValues[10];
            					}
								simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
								if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               						sensor_value[1] = auxValues[10];
            					}
								simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
								if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               						sensor_value[2] = auxValues[10];
            					}
								simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
								if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               						sensor_value[3] = auxValues[10];
            					}
            					printf("the sensor values junction***: %f %f %f %f  \n",sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3]);
								if(sensor_value[0]>0.7 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.7)			//junction
								{	//straight line
									a = a+1;
									if(a==2)
									{	a = 0;
										c =1;
										printf("ByE bYe junc\n");
										break;
									}
								}
								else{	printf("junc aaya so left turn\n");
                    		   		move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns left
								}
							  }
                    	     }
						}
					if (c == 1){
						c = 0;
						break;
					}
				}
			}
			else if(sensor_value[0]>0.6 && sensor_value[1]<0.6 && sensor_value[2]<0.6 && sensor_value[3]>0.6)			//loop for detecting dead end 
			{
				while(simxGetConnectionId(clientID)!=-1)
                {   //reading sensors
					simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[0] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[1] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[2] = auxValues[10];
            		}
					simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
					if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               			sensor_value[3] = auxValues[10];
            		}
		
            		
            		if(sensor_value[0]>0.4 && sensor_value[1]>0.3 && sensor_value[2]>0.3 && sensor_value[3]>0.4)
            		{
						while(simxGetConnectionId(clientID)!=-1){
							simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
							if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               					sensor_value[0] = auxValues[10];
            				}
							simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
							if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               						sensor_value[1] = auxValues[10];
            				}
							simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
							if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               					sensor_value[2] = auxValues[10];
            				}
							simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
							if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               				sensor_value[3] = auxValues[10];
            				}
					
            				printf("the sensor values straight-line***: %f %f %f %f   \n",sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3]);
            				if(sensor_value[0]>0.7 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.7)			
							{	//straight line
									printf("ByE bYe u-turn\n");
									break;
							}
							else{	printf("dead end aaya so left turn\n");
                    		   	move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns left
							}
						}	
            		}		
            		else 
            			break;
            		
            	 }
			 }

			//straight line
			int all_black_flag = 1;
    		float weighted_sum = 0, sum = 0, pos = 0;
    
    		for(int i = 0; i < 4; i++){
        		if(sensor_value[i] < 0.4)
        			all_black_flag = 0;
       			weighted_sum += (float)(sensor_value[i]) * (weights[i]);
        		sum += sensor_value[i];
      		}
    		if(sum != 0)
    			pos = weighted_sum / sum;

			if(all_black_flag == 1)
    		{
        		if(yaw_error > 0)
            		pos = 0.15;
        		else
            		pos = -0.15;
    		}

    		yaw_error = pos;
        	//calculating_yaw_correction :-
			//yaw_error *= 10;
    		yaw_difference = (yaw_error - yaw_prev_error);
    		yaw_cumulative_error += yaw_error;
    
    		if(yaw_cumulative_error > 0.3)
    		    yaw_cumulative_error = 0.3;
        
    		else if(yaw_cumulative_error < -0.3)
    		    yaw_cumulative_error = -0.3;
    
			yaw_correction = yaw_kP*yaw_error + yaw_kI*yaw_cumulative_error + yaw_kD*yaw_difference;
		    yaw_prev_error = yaw_error;
			
           	right_pwm = constrain((speed+ yaw_correction), MIN_PWM, MAX_PWM);
            left_pwm = constrain((speed-yaw_correction), MIN_PWM, MAX_PWM);
                       
            //Extra yaw correction during turns
            if(yaw_error>0.05)
            {
                right_pwm-=0.3;
                left_pwm+=0.3;   
            }
            else if(yaw_error<0.05)
            {
                left_pwm-=0.3;
                right_pwm+=0.3;
            }
                
         	move(clientID,leftMotorHandle,rightMotorHandle,left_pwm,right_pwm);

      
        	
        	
        }
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
