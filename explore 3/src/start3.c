#include "vrep.h"

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

//for junc_check
int cordinate[1000][2];
int type[1000]={1};
int init_direction[1000];	
int path_type[1000][3];			// 1st column-> left path  2nd column->straight path  3rd column->right path

	 
int explore[1000]={1};
int turn_stored[3000];			//to store all the turns			//10->left 20-> straight 30->right 40->u-turn
int i=-1;
int x=0,y=0,total_points=0;		//x,y -> coordinate-distance
int direction = 1;
int t;

//for bot's motion func
float turn_speed= 1;

//for old_node func
int leftMotorHandle;
int rightMotorHandle;
int clientID;  
float *read_sensors(int clientID, int sensor[]);
int encoder_value();
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float left_pwm,float right_pwm);
float constrain(float x, float lower_limit, float higher_limit);

//for turn func
int sensor[5];
												int unexplored[1000];
												int m;
//in same file
int junc_check();
int old_node(int i);
int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed, int count, float sensor_value[]);


int main(int argc,char* argv[])
{

    // all handles that will be passed from lua to this code
    int portNb=-1;
	int sensor[5];
    // number of variables passed from lua to this code + 1
    // (+1 because the first value in argv is always the location of this program)
    int n = 9;

    // save the handles
    if (argc>=n)
	{
        portNb=atoi(argv[1]); 
        leftMotorHandle = atoi(argv[2]);
        rightMotorHandle = atoi(argv[3]);
		sensor[0] = atoi(argv[4]);
		sensor[1] = atoi(argv[5]);
		sensor[2] = atoi(argv[6]);
		sensor[3] = atoi(argv[7]);
		sensor[4] = atoi(argv[8]);
    }
    else
    {
        printf("Sufficient arguments not provided'!\n");
        extApi_sleepMs(5000);
        return 0;
    }
    
    		int l;
			int junction= 0;
			int tshape=0;
			int straight_left = 0;
			int straight_right = 0;
			int pure_left = 0;
			int pure_right=0;
			int u_turn = 0;
			int unexplored_turn;
    clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	int speed = 0.1;
	float *sensor_values;
	//for reading sensor values
	if (clientID!=-1)
    {
        printf("connection established...\n");
		float time=0;
     
		while (simxGetConnectionId(clientID)!=-1)
		{	
            //reading sensors
			sensor_values = read_sensors(clientID, sensor);
		if(sensor_values[0]>0 && sensor_values[1]>0 && sensor_values[2]>0 && sensor_values[3]>0)					//all the sensor readings should be greater than 0
     	{
     		if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4 && sensor_values[4]<0.4)							//end of maze
			{
				while(simxGetConnectionId(clientID)!=-1)
           		{   //reading sensors
					sensor_values = read_sensors(clientID, sensor);
								
            		if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
					{	//straight line
						int j =-1, sign;
									sign = (direction<=2) ? 1 : (-1);
									if( direction == 1 || direction == 3)   //north direction
    									y = y + (sign)*l;     //changing y-coordinate
   									 else 
     									x = x + (sign)*l;     //changing x-coordinate
						printf("ByE bYe u-turn\n");
						++u_turn;
						if(direction == 1)
							direction = 3;
						if(direction == 2)
							direction = 4;
						else
							direction = direction - 2;
						break;
					}
					else
					{	
							printf("end of maze aaya so left turn\n");
                    	   	move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns 180 left turn
					}
				}
			}
			
			else
			{
				if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			//pure left or straight left turn
				{           
                	while(simxGetConnectionId(clientID)!=-1)
                	{   //reading sensor values
                 		sensor_values = read_sensors(clientID, sensor);
                                    
						if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)	//getting till end of node
         					move(clientID,leftMotorHandle,rightMotorHandle,turn_speed,turn_speed);
                   		else
     					{
     						if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)			//pure left
                   			{
								//finding coordinate points of the junction
   								l = encoder_value();
   								i = junc_check(l);
   								if(i != -1)		//old node
   								{	//find the nearest node		
   								}		
   								else				//new node
   								{
   									int v=0;
   									i=total_points+1;
        							cordinate[i][0]=x;
        							cordinate[i][1]=y;
     								type[i]=1;
       			 					total_points++;
       			 					path_type[i][0] = 1;
       			 					init_direction[i] = direction;	
       			 					explore[i] = 1;
       								//turn(clientID, leftMotorHandle, rightMotorHandle, -1*turn_speed, turn_speed, 0);		//turns left
       								while(simxGetConnectionId(clientID)!=-1)
		                			{   //reading sensors
										sensor_values = read_sensors(clientID, sensor);
										v = turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, 0, sensor_values);
										if(v==1)
										{
											direction = (direction==1) ? 4 : (direction-1);
												turn_stored[t] = 10;
												t++;
											break;
										
										}
									}		//turns left
       			 				}
       			 				break;
       			 			}
       			 			
       			 			else if(sensor_values[0]>0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.4)				//straight left
							{                 					
   								//finding coordinate points of the junction
    					 		l = encoder_value();
   								i = junc_check(l);		
    									
       	 						if(i != -1)			//old node	
   								{
   									int v=0;
   									unexplored_turn = old_node(i);
   									if(unexplored_turn != 0)
   									{
   									while(simxGetConnectionId(clientID)!=-1)
		                			{   //reading sensors
										sensor_values = read_sensors(clientID, sensor);
										v = turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed, path_type[i][1], sensor_values);
										if(v==1)
										{
											if(unexplored_turn < 0 )
											{	direction = (direction==1) ? 4 : (direction-1);
												turn_stored[t] = 10;
												t++;
											}
											else
											{	direction = (direction==4) ?1 : (direction+1);
												turn_stored[t] = 30;
												t++;
											}
											break;
										}
									}	
									}	//turns left
   									break;
      							}
      							else				//new node
   								{
   									int v=0;
   									i=total_points+1;
        							cordinate[i][0]=x;
        							cordinate[i][1]=y;
     								type[i]=2;
       			 					total_points++;
       			 					path_type[i][0] = 1;
       			 					path_type[i][1] = -1;
       			 					explore[i] = 1;
       			 					init_direction[i] = direction;	
       								//turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, -1);		//turns left
       								while(simxGetConnectionId(clientID)!=-1)
		                			{   //reading sensors
										sensor_values = read_sensors(clientID, sensor);
										v = turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, -1, sensor_values);
										if(v==1)
										{
											
												direction = (direction==1) ? 4 : (direction-1);
												turn_stored[t] = 10;
												t++;
											
											
											break;
										}
									}		//turns left
       			 				}
       			 				break;
       			 			}
       			 		}
       			 	}
				}	
				
				
				else if(sensor_values[3]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[0]>0.7)			//pure right or straight right turn detected
				{        
						while(simxGetConnectionId(clientID)!=-1)
		                {   //reading sensors
							sensor_values = read_sensors(clientID, sensor);
	
							if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4)
        		            	move(clientID,leftMotorHandle,rightMotorHandle,turn_speed,turn_speed);
                   
        		           else
        		           { 	                		
        		           		if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)		//pure right
								{
       								l = encoder_value();
   									i = junc_check(l);
   									if(i != -1)		//old node
   									{	//find the nearest node	
   										break;	
   									}	
   									else				//new node
   									{
   										int v=0;
   										i=total_points+1;
        								cordinate[i][0]=x;
        								cordinate[i][1]=y;
     									type[i]=1;
       			 						total_points++;
       			 						path_type[i][0] = 1;
       			 						init_direction[i] = direction;	
       			 						explore[i] = 1;
       									//turn(clientID, leftMotorHandle, rightMotorHandle, turn_speed, -1*turn_speed, 0);		//turns right
       									while(simxGetConnectionId(clientID)!=-1)
		                				{   //reading sensors
										sensor_values = read_sensors(clientID, sensor);
										v = turn(clientID, leftMotorHandle, rightMotorHandle, turn_speed, -1*turn_speed, 0, sensor_values);
										if(v==1)
										{
											
												direction = (direction==4) ?1 : (direction+1);
												turn_stored[t] = 30;
												t++;
											
											break;
										}
										}		//turns left
       			 					}
       			 					break;
       			 				}
       			 				
       			 				else if(sensor_values[0]>0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.4)			//straight-right node
                      			{
                      				//finding coordinate points of the junction
    					 			l = encoder_value();
   									i = junc_check(l);			
       			 					
       			 					if(i != -1)		//old node
       			 					{
       			 						int v=0;
   										unexplored_turn = old_node(i);
   										if(unexplored_turn !=0)
   										{
   										while(simxGetConnectionId(clientID)!=-1)
		                				{   //reading sensors
											sensor_values = read_sensors(clientID, sensor);
											v = turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed, path_type[i][1], sensor_values);
											if(v==1)
											{
												if(unexplored_turn < 0 )
											{	direction = (direction==1) ? 4 : (direction-1);
												turn_stored[t] = 10;
												t++;
											}
											else
											{	direction = (direction==4) ?1 : (direction+1);
												turn_stored[t] = 30;
												t++;
											}
												break;
											}
										}
										}		//turns left
   										break;
      								}
      								else				//new node
   									{
   										i=total_points+1;
        								cordinate[i][0]=x;
        								cordinate[i][1]=y;
     									type[i]=2;
       			 						total_points++;
       			 						path_type[i][1] = 1;
       			 						path_type[i][2] = -1;
       			 						explore[i] = 1;
       			 						init_direction[i] = direction;
       			 						turn_stored[t] = 20;
   										t++;	
       									break;
       								}
       			 				}
       			 			}
       			 		}
       			 	}
       			 	
       			 	
       			 	else if(sensor_values[3]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[0]<0.4)					//for junction or t-shape
					{
						while(simxGetConnectionId(clientID)!=-1)
                		{   //reading sensors
							sensor_values = read_sensors(clientID, sensor);

						if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4)
						{	printf("j/t khtm hone tak straight\n");
                    		move(clientID,leftMotorHandle,rightMotorHandle,turn_speed,turn_speed);
						}
						else
						{
							if(sensor_values[0]>0.4 && sensor_values[1]>0.4 && sensor_values[2]>0.4 && sensor_values[3]>0.4)		//t-shape	
							{
								//finding coordinate points of the junction
    					 		l = encoder_value();
   								i = junc_check(l);			
       			 					
       			 				if(i != -1)		//old node
       			 				{
       			 					int v=0;
   									unexplored_turn = old_node(i);
   									if(unexplored_turn !=0)
   									{
   									while(simxGetConnectionId(clientID)!=-1)
		                			{   //reading sensors
										sensor_values = read_sensors(clientID, sensor);
										v = turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed,1, sensor_values);
										if(v==1)
										{
											if(unexplored_turn < 0 )
											{	direction = (direction==1) ? 4 : (direction-1);
												turn_stored[t] = 10;
												t++;
											}
											else
											{	direction = (direction==4) ?1 : (direction+1);
												turn_stored[t] = 30;
												t++;
											}
											break;
										}
									}		//turns left
									}
   									break;
       			 				}
       			 				else				//new node
   								{
   										int v=0;
   										i=total_points+1;
        								cordinate[i][0]=x;
        								cordinate[i][1]=y;
     									type[i]=2;
       			 						total_points++;
       			 						path_type[i][0] = 1;
       			 						path_type[i][2] = -1;
       			 						explore[i] = 1;
       			 						init_direction[i] = direction;
       			 						//turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, 0);		//turns left
       			 						while(simxGetConnectionId(clientID)!=-1)
		                				{   //reading sensors
										sensor_values = read_sensors(clientID, sensor);
										v = turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, 0, sensor_values);
										if(v==1)
										{
											direction = (direction==1) ? 4 : (direction-1);
												turn_stored[t] = 10;
												t++;
											break;
										}
										}		//turns left
       							}
       							break;
       						}
       						
       						else if(sensor_values[0]>0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.4)				//junction
							{
								//finding coordinate points of the junction
    					 		l = encoder_value();
   								i = junc_check(l);			
       			 					
       			 				if(i != -1)		//old node
       			 				{
       			 					int v=0;
   									unexplored_turn = old_node(i);
   									if (unexplored_turn !=0)
   									{
   									while(simxGetConnectionId(clientID)!=-1)
		                			{   //reading sensors
										sensor_values = read_sensors(clientID, sensor);
										v = turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed, 1, sensor_values);
										if(v==1)
										{
											if(unexplored_turn < 0 )
											{	direction = (direction==1) ? 4 : (direction-1);
												turn_stored[t] = 10;
												t++;
											}
											else
											{	direction = (direction==4) ?1 : (direction+1);
												turn_stored[t] = 30;
												t++;
											}
											break;
										}
									}	
									}	//turns left
   									break;
       			 				}
       			 				else				//new node
   								{		int v=0;
   										i=total_points+1;
        								cordinate[i][0]=x;
        								cordinate[i][1]=y;
     									type[i]=3;
       			 						total_points++;
       			 						path_type[i][0] = 1;
       			 						path_type[i][1] = -1;
       			 						path_type[i][2] = -1;
       			 						explore[i] = 1;
       			 						init_direction[i] = direction;
       			 						//turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, -1);		//turns left
       			 						while(simxGetConnectionId(clientID)!=-1)
		                				{   //reading sensors
										sensor_values = read_sensors(clientID, sensor);
										v = turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, -1, sensor_values);
										if(v==1)
										{
											direction = (direction==1) ? 4 : (direction-1);
												turn_stored[t] = 10;
												t++;
											break;
										}
										}		//turns left
       									
       							}
       							break;
       						}
       					}
       				}
       			}
       			
       			
       			else if(sensor_values[0]>0.6 && sensor_values[1]<0.6 && sensor_values[2]<0.6 && sensor_values[3]>0.6)			//loop for detecting dead end 
				{
					while(simxGetConnectionId(clientID)!=-1)
            		{   //reading sensors
						sensor_values = read_sensors(clientID, sensor);
		
            			if(sensor_values[0]>0.4 && sensor_values[1]>0.3 && sensor_values[2]>0.3 && sensor_values[3]>0.4)
            			{
							while(simxGetConnectionId(clientID)!=-1)
							{
								sensor_values = read_sensors(clientID, sensor);
            					if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
								{	//straight line
									int j =-1, sign;
									sign = (direction<=2) ? 1 : (-1);
									if( direction == 1 || direction == 3)   //north direction
    									y = y + (sign)*l;     //changing y-coordinate
   									 else 
     									x = x + (sign)*l;     //changing x-coordinate
									if(direction == 1)
										direction = 3;
									else if(direction == 2)
										direction = 4;
									else
										direction = direction - 2;
									turn_stored[t] = 40;
									t++;
									printf("ByE bYe u-turn\n");
									++u_turn;
									break;
								}
								else
								{	
									printf("dead end aaya so left turn\n");
            		        	   	move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns 180 left turn
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
    	 	
    		 	for(int i = 0; i < 4; i++)
    		 	{
    	    		if(sensor_values[i] < 0.4)
        				all_black_flag = 0;
       				weighted_sum += (float)(sensor_values[i]) * (weights[i]);
       				sum += sensor_values[i];
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
		}   
    	// end communication with lua
    	simxFinish(clientID);
	
    }
    printf("junction: %d\n", junction);
    printf("T-shape: %d\n", tshape);
    printf("straight left: %d\n", straight_left);
    printf("pure left: %d\n", pure_left);
    printf("straight right: %d\n", straight_right);
    printf("pure right: %d\n", pure_right);
	printf("u-turn: %d\n", u_turn);
	for(i = 0;i<=total_points;i++)
	{
		printf("co-ordinate point: %d,%d  explore_paths: %d   type_path: %d initial_direction: %d  path_type: %d %d %d\n",cordinate[i][0],cordinate[i][1],explore[i],type[i],init_direction[i], path_type[i][0], path_type[i][1], path_type[i][2]);
	}
	printf("final direction:%d \n", direction);
	for(i = 0;i<m;i++)
	{
		printf("unexplored_turns = %d \n", unexplored[i]);
	}
	printf("t= %d\n", t);
	for(i = 0;i<t;i++)
	{
		printf("turns stored = %d \n", turn_stored[t]);
	}
    printf("Simulation Stopped!");

}



float *read_sensors(int clientID, int sensor[])
{
		static float sensor_value[5];
		float* auxValues = NULL;
    	int* auxValuesCount = NULL;
    	
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
        simxReadVisionSensor(clientID, sensor[4], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
		if( simxReadVisionSensor(clientID, sensor[4], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
            sensor_value[4] = auxValues[10];
        }
        printf("the sensor values: %f %f %f %f %f  %d\n",sensor_value[4], sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3],direction);
        return sensor_value;
        
}

float constrain(float x, float lower_limit, float higher_limit)
{
    if(x < lower_limit)
        x = lower_limit;
    
    else if(x > higher_limit)
        x = higher_limit;

    return x;
}


int junc_check(int l)
{
	int j =-1, sign;
	sign = (direction<=2) ? 1 : (-1);
	if( direction == 1 || direction == 3)   //north direction
    	y = y + (sign)*l;     //changing y-coordinate
    else 
     	x = x + (sign)*l;     //changing x-coordinate
    
   	for( int f=1;f<=total_points;f++)
    {
    	if(x== cordinate[f][0] && y== cordinate[f][1])
       	{
      		j = f;
       		break;
      	}
   	}
   	return j;
}

int encoder_value()
{
	return 5;
}

int move(int clientID,int leftMotorHandle,int rightMotorHandle,float left_pwm,float right_pwm)
{
	simxSetJointTargetVelocity(clientID,leftMotorHandle,left_pwm,simx_opmode_oneshot);
	simxSetJointTargetVelocity(clientID,rightMotorHandle,right_pwm,simx_opmode_oneshot);
}


/*int turn_left(int clientID, int leftMotorHandle,int rightMotorHandle, int count=1)
{
	int a=0;
	while(simxGetConnectionId(clientID)!=-1)
	{
		//reading sensors
		sensor_values = read_sensors(clientID, sensor);
		if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
		{	//straight line
			if(count == 2)
			{
			a = a+1;
			if(a==2)
			{	
				++junction;
				direction = (direction==1)?4 : (direction-1);
				turn_stored[t] = 'l';
   				t++;
				break;
			}
			}
			
			if(count == 1)
			{
				++junction;
				direction = (direction==1)?4 : (direction-1);
				turn_stored[t] = 'l';
   				t++;
				break;
			}
		}	
		else	
            move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns left
	//simxSetJointTargetVelocity(clientID,leftMotorHandle,-1*turn_speed,simx_opmode_oneshot);
	//simxSetJointTargetVelocity(clientID,rightMotorHandle,turn_speed,simx_opmode_oneshot);
}*/

/*int turn_right(int clientID, int leftMotorHandle,int rightMotorHandle, int count=1)
{
	int a=0;
	while(simxGetConnectionId(clientID)!=-1)
	{
		//reading sensors
		sensor_values = read_sensors(clientID, sensor);
		if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
		{	//straight line
			if(count == 2)
			{
			a = a+1;
			if(a==2)
			{	
				++junction;
				direction = (direction==4)?1 : (direction+1);
				turn_stored[t] = 'r';
   				t++;
				break;
			}
			}
			
			if(count == 1)
			{
				++junction;
				direction = (direction==4)?1 : (direction+1);
				turn_stored[t] = 'r';
   				t++;
				break;
			}
		}	
		else	
            move(clientID,leftMotorHandle,rightMotorHandle,turn_speed,-1*turn_speed);		//turns left
	//simxSetJointTargetVelocity(clientID,leftMotorHandle,-1*turn_speed,simx_opmode_oneshot);
	//simxSetJointTargetVelocity(clientID,rightMotorHandle,turn_speed,simx_opmode_oneshot);
}*/



/*
//for junction

										
										l = encoder_value();
   										i = junc_check();
   										
   										if(i != -1)			//old node	
   										{
   											int dl=-1,dr=-1;
   											dir = init_direction[i];		//initial direction of bot on that node
   											dl = (dir==1)? 4 : (dir-1);
   											dr = (dir==4)? 1 :	(dir+1);		
   											if(direction == dir)
   											{	
   												if(explore[i] < type[i] )
   												{
   					   								//decide the turn which has to be taken to explore the unexplored path
   													if(path_types[i][1] == 0)
   													{
   														explore[i]+=1;
   														path_types[i][1] = 1;
   														turn_stored[t] = 's';
   														t++;
   														break;
   													}
   													else if(path_types[i][2] == 0)
   													{
   														explore[i]+=1;
   														path_types[i][2] = 1;
   														turn_right(clientID, leftMotorHandle, rightMotorHandle, 2);
   													}
   												}
   											}
   											else if((direction - dir) == 2 || (direction-dir)==(-2))
   											{	//bot has explore the straight path
   												int add;
   												add = turn_types[i][1] == 0 ? 1 : 0;
   												explore[i] = explore[i] + add;
   												turn_types[i][1] = 1;
   												if(explore[i] < type[i]
   												{
   													//turn left (right turn w.r.t bot's initial direction)
   													explore[i]+=1;
   													path_types[i][2] = 1;
   													turn_left(clientID, leftMotorHandle, rightMotorHandle, 2);
   												}
   											}
   											else if(direction == dl)
   											{	//bot has explore the right path
   												int add;
   												add = turn_types[i][2] == 0 ? 1 : 0;
   												explore[i] = explore[i] + add;
   												if(explore[i] < type[i] )
   												{
   													//turn right (straight turn w.r.t bot's initial direction)
   													explore[i]+=1;
   													path_types[i][1] = 1;
   													turn_right(clientID, leftMotorHandle, rightMotorHandle, 2);
   												}
   											}
   											else if(direction == dr)
   											{
   												//bot has explore the left path
   												if(explore[i] < type[i] )
   												{
   													if(path_type[i][1] == 0)
   													{
   														//turn left (straight turn w.r.t bot's initial direction)
   														explore[i]+=1;
   														path_types[i][1] = 1;
   														turn_left(clientID, leftMotorHandle, rightMotorHandle, 2);
   													}
   													else if(path_type[i][2] == 0)
   													{
   														//straight path (right turn w.r.t.bot's initial direction)
   														explore[i]+=1;
   														path_types[i][2] = 1;
   														turn_stored[t] = 's';
   														t++;
   														break;
   													}
   												}
   											}
   											
*/
   		
   										
   										
   										
   										
   										
//general
int old_node(int i)
{
											int dl=-1,dr=-1,dir;
   											dir = init_direction[i];		//initial direction of bot on that node
   											dl = (dir==1)? 4 : (dir-1);
   											//dr = (dir==4)? 1 :	(dir+1);
   											
   											//for getting the path from where the bot has come
   											if((direction - dir) == 2 || (direction-dir)==(-2))
   											{	//bot has explore the straight path
   												if (path_type[i][1] == -1)
   													explore[i]+=1;
   												path_type[i][1] = 1;
   											}
   											else if(direction == dl)
   											{	//bot has explore the right path
   												if (path_type[i][2] == -1)
   													explore[i]+=1;
   												  												path_type[i][2]=1;
   											}
   											
   											
   											
   											//for getting the path where the bot has to move
   											int unexplored_turn=0,target=0;
   											if(explore[i] < type[i])
   											{
   												if(path_type[i][1] == -1)
   												{
   													unexplored_turn = (dir - direction);
   													explore[i]+=1;
   													path_type[i][1] = 1;
   												}
   												else if(path_type[i][2] == -1)
   												{
   													target = (dir==4)?1:(dir+1);	//jaha jana h
   													unexplored_turn = target - direction;
   													explore[i]+=1;
   													path_type[i][2] = 1;
   												}	
   												/*if(unexplored_turn == -1)
   												{
   														turn_stored[t] = 's';
   														t++;
   												}*/
   												if(unexplored_turn == 0)
   												{
   														turn_stored[t] = 's';
   														t++;
   												}
   												if(unexplored_turn== -3 || unexplored_turn == 3)
   													unexplored_turn = -1*unexplored_turn/3;
   												//turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed, path_type[i][1]); 
   																					
   											}	
   											else if(explore[i] == type[i])
   											{
   												//finding nearest node
   											}
   											unexplored[m] = unexplored_turn;
   											m++;
   											return  unexplored_turn;
}  												
   												
int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed, int count, float sensor_value[])
{
	int a=0;
	int v=0;
	//float *sensor_value;
	//while(simxGetConnectionId(clientID)!=-1)
	//{
		//reading sensors
		//sensor_value = read_sensors(clientID, sensor);
		if(sensor_value[0]>0.7 && sensor_value[1]<0.4 && sensor_value[2]<0.4 && sensor_value[3]>0.7)			
		{	//straight line
			if(count ==-1 || count==1)
			{
			a = a+1;
			if(a==2)
			{	
					/*if (leftMotorSpeed<0)
					{	direction = (direction==1)?4 : (direction-1);
						turn_stored[t] = 'l';
   						t++;
   					}
   					else
   					{	direction = (direction==4)?1 : (direction+1);
						turn_stored[t] = 'r';
   						t++;
   					}*/
				//break;
			}
			}
			
			else if(count == 0)
			{
					/*if (leftMotorSpeed<0)
					{	direction = (direction==1)?4 : (direction-1);
						turn_stored[t] = 'l';
   						t++;
   					}
   					else
   					{	direction = (direction==4)?1 : (direction+1);
						turn_stored[t] = 'r';
   						t++;
   					}*/
				//break;
			}
			v = 1;
		}	
		else	
            move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,rightMotorSpeed);		//turns left
	//simxSetJointTargetVelocity(clientID,leftMotorHandle,-1*turn_speed,simx_opmode_oneshot);
	//simxSetJointTargetVelocity(clientID,rightMotorHandle,turn_speed,simx_opmode_oneshot);
	//}
	return v;
} 												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
