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
float yaw_kI= 0.1;
float yaw_kD= 0.1;

//FOR LINE FOLLOWING
float yaw_error=0, yaw_prev_error=0, yaw_difference=0, yaw_cumulative_error=0, yaw_correction=0;
int weights[4] = {3,1,-1,-3};
float left_pwm = 0, right_pwm = 0;
int pid_control(float sensor_values[]);
int pid_correction(float sensor_values[], int index);

//for junc_check
int cordinate[100][2];
int all_cordinate[100][2];
int all_cordinate_index;
int type[100]={1};
int init_direction[100]={1};	
int path_type[100][3];			// 1st column-> left path  2nd column->straight path  3rd column->right path

	 
int explore[100]={1};

int i=-1;
int x=0,y=0,total_points=0;		//x,y -> coordinate-distance
int direction = 1;

									int bhu;
									int expo;
//for bot's motion func
float turn_speed=1;

//for old_node func
int leftMotorHandle;
int rightMotorHandle;
int clientID;  
float *read_sensors(int clientID, int sensor[]);
int encoder_value();
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float left_pwm,float right_pwm);
float constrain(float x, float lower_limit, float higher_limit);

			int check_all[100] = {0};
			int abc;
//for turn func
int sensor[5];
int b;											
												int unexplored[100]={0};
												int m;
//in same file
int junc_check();
int old_node(int i, int expo, int turn_stored[],int t);
int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed, int count);
int find(int explore[],int type[],int cordinate[][2],int k,int total_points);		//pratam
int travel(int z,int cordinate[][2],int turn_stored[],int t, int d, int expo); 		//pratam



int main(int argc,char* argv[])
{

    // all handles that will be passed from lua to this code
    int portNb=-1;
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
    	int turn_stored[300];			//to store all the turns			//10->left 20-> straight 30->right 40->u-turn
    	int t=0;
    		int l;
			int junction= 0;
			int tshape=0;
			int straight_left = 0;
			int straight_right = 0;
			int pure_left = 0;
			int pure_right=0;
			int u_turn = 0;
			int unexplored_turn;
			int node_detected=0;
    clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	int speed =0 ;
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
     		if(sensor_values[0]<0.7 && sensor_values[1]<0.7 && sensor_values[2]<0.7 && sensor_values[3]<0.7 && sensor_values[4]<0.7)							//end of maze
			{
				while(simxGetConnectionId(clientID)!=-1)
           		{   //reading sensors
					sensor_values = read_sensors(clientID, sensor);
								
            		if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
					{	//straight line
						int j =-1, sign;
						sign = (direction<=2) ? (1) : (-1);
						if( direction == 1 || direction == 3)   //north direction
    						y = y + (sign)*l;     //changing y-coordinate
   						else 
     						x = x + (sign)*l;     //changing x-coordinate
     								all_cordinate[all_cordinate_index][0] = x;
     								all_cordinate[all_cordinate_index][1] = y;
     								all_cordinate_index+=1;
     					direction = (direction>2)?(direction-2) : direction+2;
     					node_detected = 1;
						printf("ByE bYe u-turn\n");
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
         				{	printf("l/sl khtm hone tak straight\n");
         					pid_correction(sensor_values,0);
         				}
                   		else
     					{
     						if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)			//pure left
                   			{
								//finding coordinate points of the junction
   								l = encoder_value();
   								i = junc_check(l);
   								if(i != -1)		 //old node
   								{	//find the nearest node		
   									while(simxGetConnectionId(clientID)!=-1)
		                				{   //reading sensors
											sensor_values = read_sensors(clientID, sensor);
											if(sensor_values[0] > 0.5 && sensor_values[1] < 0.5 && sensor_values[2] < 0.5 && sensor_values[3] > 0.5 )
												break;
											else
												move(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
										}
										direction = (direction==1)?4:(direction-1);
										turn_stored[t] = 10;
										t++;
										node_detected = 1;
   										break;	
   								}		
   								else				//new node
   								{
   									
   									i=total_points+1;
        							cordinate[i][0]=x;
        							cordinate[i][1]=y;
     								type[i]=1;
       			 					total_points++;
       			 					path_type[i][0] = 1;
       			 					init_direction[i] = direction;	
       			 					explore[i] = 1;
       								
       								
									turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, 0);
										
									direction = (direction==1) ? 4 : (direction-1);
									turn_stored[t] = 10;
									t++;
									node_detected = 1;
									break;
										
										
											//turns left
       			 				}
       			 				break;
       			 			}
       			 			
       			 			else if(sensor_values[0]>0.7 && sensor_values[1]<0.5 && sensor_values[2]<0.5 && sensor_values[3]>0.7)				//straight left
							{                 					
   								//finding coordinate points of the junction
    					 		l = encoder_value();
   								i = junc_check(l);		
    									
       	 						if(i != -1)			//old node	
   								{
   								
   									unexplored_turn = old_node(i,100,turn_stored,t);
   									if(unexplored_turn != 20 && unexplored_turn != -2)
   									{
   										turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed,1);
										
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
									
									}	//turns left
									else if (unexplored_turn == 20)
   												{
   														
   														turn_stored[t] = 20;
   														t++;
   												}
   									node_detected = 1;
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
       								
       								turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, -1);
																		
									direction = (direction==1) ? 4 : (direction-1);
									turn_stored[t] = 10;
									t++;
									node_detected = 1;
									break;
									
											//turns left
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
							{	printf("r/sr khtm hone tak straight\n");
        		            	pid_correction(sensor_values,3);
                   			}
        		           else
        		           { 	                		
        		           		if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)		//pure right
								{
									l = encoder_value();
									i = junc_check(l);
   									if(i != -1)		//old node
   									{	//finding nearest node
   										
										while(simxGetConnectionId(clientID)!=-1)
		                				{   //reading sensors
											sensor_values = read_sensors(clientID, sensor);
											if(sensor_values[0] > 0.7 && sensor_values[1] < 0.3 && sensor_values[2] < 0.3 && sensor_values[3] > 0.7 )
												break;
											else
												move(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
										}
										direction = (direction==4)?1:(direction+1);
										turn_stored[t] = 30;
										t++;
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
       									
       									turn(clientID, leftMotorHandle, rightMotorHandle, turn_speed, -1*turn_speed, 0);
										
										direction = (direction==4) ?1 : (direction+1);
										turn_stored[t] = 30;
										t++;														
										//turns left
       			 					}
       			 					node_detected = 1;
       			 					break;
       			 				}
       			 				
       			 				else if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			//straight-right node
                      			{
                      				//finding coordinate points of the junction
    					 			l = encoder_value();
   									i = junc_check(l);			
       			 					
       			 					if(i != -1)		//old node
       			 					{
       			 						int v=0;
   										unexplored_turn = old_node(i, 200,turn_stored,t);
   										if(unexplored_turn != 20 && unexplored_turn != -2)
   										{
											turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed, 1);
																						
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
										}		//turns left
   										if(unexplored_turn == 20)
   												{
   														
   														turn_stored[t] = 20;
   														t++;
   												}
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
   										node_detected = 1;
       									break;
       								}
       			 				}
       			 			}
       			 		}
       			 	}
       			 	
       			 	
       			 	else if(sensor_values[3]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[0]<0.3)					//for junction or t-shape
					{
						while(simxGetConnectionId(clientID)!=-1)
                		{   //reading sensors
							sensor_values = read_sensors(clientID, sensor);

						if(sensor_values[0]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[3]<0.3)
						{	printf("j/t khtm hone tak straight\n");
                    		move(clientID, leftMotorHandle, rightMotorHandle, turn_speed, turn_speed);
						}
						else
						{
							if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)		//t-shape	
							{
								//finding coordinate points of the junction
    					 		l = encoder_value();
   								i = junc_check(l);			
       			 					
       			 				if(i != -1)		//old node
       			 				{
       			 					int v=0;
   									unexplored_turn = old_node(i, 300,turn_stored,t);
   									if(unexplored_turn != 20 && unexplored_turn != -2)
   									{
   									
										turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed,1);
										
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
											
									}
   									if(unexplored_turn == 20)
   												{
   														
   														turn_stored[t] = 20;
   														t++;
   												}
   								node_detected = 1;
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
       			 						
       			 						turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, 0);
										direction = (direction==1) ? 4 : (direction-1);
										turn_stored[t] = 10;
										node_detected = 1;
										t++;
											
       							}
       							break;
       						}
       						
       						else if(sensor_values[0]>0.7 && sensor_values[1]<0.7 && sensor_values[2]<0.7 && sensor_values[3]>0.7)				//junction
							{
								//finding coordinate points of the junction
    					 		l = encoder_value();
   								i = junc_check(l);			
       			 					
       			 				if(i != -1)		//old node
       			 				{
       			 					int v=0;
   									unexplored_turn = old_node(i, 400,turn_stored,t);
   									if (unexplored_turn != 20 && unexplored_turn != -2)
   									{
   									
										turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed, 1);
										
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
											
									}	//turns left
   									if(unexplored_turn == 20)
   												{
   														
   														turn_stored[t] = 20;
   														t++;
   												}
   									node_detected = 1;
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
       			 						
										turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed, -1);
										
											direction = (direction==1) ? 4 : (direction-1);
												turn_stored[t] = 10;
												t++;
										node_detected = 1;
											
       							}
       							break;
       						}
       					}
       				}
       			}
       			
       			
       			else if(sensor_values[0]>0.6 && sensor_values[1]<0.6 && sensor_values[2]<0.6 && sensor_values[3]>0.6)			//loop for detecting dead end 
				{
					printf("deadend ke andar checking if aage end aaya kya \n");
					while(simxGetConnectionId(clientID)!=-1)
            		{   //reading sensors
						sensor_values = read_sensors(clientID, sensor);
						int count=0;
            			if(sensor_values[0]>0.5 && sensor_values[1]>0.2 && sensor_values[2]>0.2 && sensor_values[3]>0.5)
            			{
            			
            				while(1)
							{
									move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);
									count++;
									if(count==600000)
									{
										bhu=3000;
										count=0;
										break;
									}
							}	
							while(simxGetConnectionId(clientID)!=-1)
							{
								sensor_values = read_sensors(clientID, sensor);
            					if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
								{	//straight line
									int j =-1, sign;
									l = encoder_value();
                   // direction=(direction+2)%4;
                   // if(direction==0)
                   // direction=4;
                  
									sign = (direction<=2) ? 1 : (-1);
									if( direction == 1 || direction == 3)   //north direction
    									y = y + (sign)*l;     //changing y-coordinate
   									 else 
     									x = x + (sign)*l;     //changing x-coordinate
     								all_cordinate[all_cordinate_index][0] = x;
     								all_cordinate[all_cordinate_index][1] = y;
     								all_cordinate_index+=1;
									direction = (direction>2)?(direction-2):(direction+2);
									node_detected = 1;
									//turn_stored[t] = 40;
									//t++;
									printf("ByE bYe u-turn\n");
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
				pid_control(sensor_values);
        		if(node_detected == 1)
        		{
        			int count_1=0;
        			while(count_1<=5000)
        			{
        				pid_control(sensor_values);
        				count_1+=1;
        			}
        			node_detected=0;
        		}
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
		printf("turns stored = %d \n", turn_stored[i]);
	}
	printf("bhu= %d \n", bhu);
	for(i=0;i<all_cordinate_index;i++)
	{
		printf("x= %d y=%d \n", all_cordinate[i][0], all_cordinate[i][1]);
	}
	for(int p = 0;p<abc;p++)
	{
		printf("retrasing values= %d \n ", check_all[p]);
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

float constrain(float k, float lower_limit, float higher_limit)
{
    if(k < lower_limit)
        k = lower_limit;
    
    else if(k > higher_limit)
        k = higher_limit;

    return k;
}


int junc_check(int l)
{
	int j =-1, sign;
	sign = (direction<=2) ? 1 : (-1);
	if( direction == 1 || direction == 3)   //north direction
    	y = y + (sign)*l;     //changing y-coordinate
    else 
     	x = x + (sign)*l;     //changing x-coordinate
    all_cordinate[all_cordinate_index][0] = x;
    all_cordinate[all_cordinate_index][1] = y;
   	all_cordinate_index+=1;
    
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

   										
//general
int old_node(int i, int expo, int turn_stored[], int t)
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
   											int unexplored_turn=-2,target=0;
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
   												if(unexplored_turn == 0)
   													unexplored_turn = 20;
   												else if(unexplored_turn== -3 || unexplored_turn == 3)
   													unexplored_turn = -1*unexplored_turn/3;									
   											}	
   											else if(explore[i] == type[i])
   											{
   													bhu=4;
	   												//finding nearest node                  
													int z;
													z= find( explore, type, cordinate, i , total_points);//block for finding cordinate;
													i = travel( z, cordinate,turn_stored, t,i,expo);
   											}
   											unexplored[m] = unexplored_turn;
   											m++;
   											return  unexplored_turn;
}  												
   												
int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed, int count)
{
	int v=0;
	float *sensor_values;
	while(1)
	{
		//straight line
			move(clientID,leftMotorHandle,rightMotorHandle,0.01,0.01);
           	
		count++;
		if(count==10)
		{
			count=0;
			break;
		}
	}
	while(1)
		{
			move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,rightMotorSpeed);
			count++;
			if(count==600000)
			{
				bhu=3000;
				count=0;
				break;
			}
		}	
		while(simxGetConnectionId(clientID)!=-1)
 		{
 			bhu=500;
 			sensor_values = read_sensors(clientID, sensor);
 			if(sensor_values[0]>0.5 &&sensor_values[1]<0.5 &&sensor_values[2]<0.5 &&sensor_values[3]>0.5)
 				break;
 			
 			else
			move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,rightMotorSpeed);
		}

} 												
   												
   												
int find(int explore[],int type[],int cordinate[][2],int k,int total_points)
{
	int x,y,x1,y1,min,final;
	x=cordinate[k][0];
	y=cordinate[k][1];
	min=1000000;
	bhu= 150;
	for(int a=0;a< total_points;a++)
	{
		if(a!=k)
		{
			if(explore[a]<type[a])
			{
	  			x1=cordinate[a][0];
	  			y1=cordinate[a][1];
	  			int xdiff,ydiff;		  
	  			xdiff=(x1-x); 
	  			ydiff=(y1-y);
				if(xdiff<0)
	    			xdiff=xdiff*(-1);
				if(ydiff<0)
		    		ydiff=ydiff*(-1);
				if((xdiff+ydiff)<min)
				{
					min=(xdiff+ydiff);
					final=a;
				}
			}	
		}
	}
	return final;
}
				
						
int travel(int z,int cordinate[][2],int turn_stored[],int t, int d, int expo)
{
    float *sensor_values;
	int v;
	float turn_speed=1;
	int speed=0;
	int x2,y2,x1,y1,c,count,l;
	bhu=300;
	count=0;
	c=0;
	int ab=0;
	x1=cordinate[z][0];
	y1=cordinate[z][1];
	y2=cordinate[d][1];
	x2=cordinate[d][0];
	
	if(expo==100)
	{
		int bb=0;
		while(1)
		{
			move(clientID,leftMotorHandle,rightMotorHandle, turn_speed ,-1*turn_speed);
			count++;
			if(count==1000000)
			{
				bhu=3000;
				count=0;
				break;
			}
		}	
		while(simxGetConnectionId(clientID)!=-1)
 		{
 			bhu=500;
 			sensor_values = read_sensors(clientID, sensor);
 			if(sensor_values[0]>0.4&&sensor_values[1]<0.4&&sensor_values[2]<0.4&&sensor_values[3]>0.4)
 			{	
 				bhu=1000;
 				direction = (direction>2)?(direction-2):(direction+2);
 				//turn_stored[t] = 40;
				//t++;
 				break;
 			}
 			else
 				move(clientID,leftMotorHandle,rightMotorHandle, turn_speed ,-1*turn_speed);  
		}
	}

	if(expo==200)
	{
		int bb=0;
		while(1)
		{
			move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed ,turn_speed);
			count++;
			if(count==1000000)
			{
				bhu=3000;
				count=0;
				break;
			}
		}	
 		while(simxGetConnectionId(clientID)!=-1)
 		{
 			//printf("dekho mein aa gya\n");
 			sensor_values = read_sensors(clientID, sensor);
 			if(sensor_values[0]>0.4&&sensor_values[1]<0.4&&sensor_values[2]<0.4&&sensor_values[3]>0.4)
 			{
 				
 					direction = (direction>2)?(direction-2):(direction+2);
 				//turn_stored[t] = 40;
				//t++;
 					break;
 				
 			}
 			else
				move(clientID,leftMotorHandle,rightMotorHandle,-1* turn_speed ,turn_speed);  
 		}
	}

	if(expo==300 || expo==400)
	{     
		while(1)
		{
			move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed ,turn_speed);
			count++;
			if(count==1000000)
			{
				bhu=3000;
				count=0;
				break;
			}
		}	
        
        while(simxGetConnectionId(clientID)!=-1)
        {
   			sensor_values = read_sensors(clientID, sensor);
   
			if(sensor_values[0]>0.4&&sensor_values[1]<0.4&&sensor_values[2]<0.4&&sensor_values[3]>0.4)
			{  
				if(expo == 400)
				{
   					c++;
   					if(c==2)
   					{
     					c=0;
     					direction = (direction>2)?(direction-2):(direction+2);
 						//turn_stored[t] = 40;
						//t++;
     					break;
   					}
   				}
   				else
   				{
   					direction = (direction>2)?(direction-2):(direction+2);
 					//turn_stored[t] = 40;
					//t++;
 					break;
   				}
			}
			else
				move(clientID,leftMotorHandle,rightMotorHandle, -1*turn_speed ,turn_speed);
		}
	}
	int lap_time=0;
	int turn_taken=-1;
	while(simxGetConnectionId(clientID)!=-1)
	{
				sensor_values = read_sensors(clientID, sensor);
				//straight line
				pid_control(sensor_values);
        		
        		while(lap_time< 5000 && turn_taken==1)
        		{
        			lap_time++;
        			pid_control(sensor_values);
        			
        		}
        		turn_taken=0;
		   
	ab = t-1;
	/*if(sensor_values[0]<0.5 && sensor_values[1]<0.5 && sensor_values[2]<0.5 && sensor_values[3]>0.6)
	{	
		while(simxGetConnectionId(clientID)!=-1)
		{
			sensor_values = read_sensors(clientID, sensor);
			if((sensor_values[0]>0.5 && sensor_values[1]>0.5 && sensor_values[2]>0.5 && sensor_values[3]>0.5) || (sensor_values[3]>0.4 && sensor_values[1]>0.4 && sensor_values[2]>0.4 && sensor_values[0]>0.4))	
			{
				check_all[abc] = turn_stored[ab];
				abc++;
				
      			if(turn_stored[ab]==30)
				{ 
					ab--;
					turn_taken=1;
					int j =-1, sign;
					l = encoder_value();
					sign = (direction<=2) ? 1 : (-1);
						if( direction == 1 || direction == 3)   //north direction
    					y2 = y2 + (sign)*l;     //changing y-coordinate
    				else 
     					x2 = x2 + (sign)*l;     //changing x-coordinate
						all_cordinate[all_cordinate_index][0] = x2;
					all_cordinate[all_cordinate_index][1] = y2;
				   	all_cordinate_index+=1;	
					if(x2==x1 && y2==y1)
						return z;
    				turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed, 0);
					direction=(direction==1)?4:(direction-1);
					break;
				}


 				else if(turn_stored[ab]==10)
				{ 
					ab--;
					turn_taken=1;
					int j =-1, sign;
					l = encoder_value();
					sign = (direction<=2) ? 1 : (-1);
					if( direction == 1 || direction == 3)   //north direction
    						y2 = y2 + (sign)*l;     //changing y-coordinate
    				else 
     					x2 = x2 + (sign)*l;     //changing x-coordinate
					all_cordinate[all_cordinate_index][0] = x2;
					all_cordinate[all_cordinate_index][1] = y2;
				   	all_cordinate_index+=1;	
					if(x2==x1 && y2==y1)
						return z;
		   			turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed, 0);
					direction=(direction==4)?1:(direction+1);
					break;
				}


 				else if(turn_stored[ab]==20)
				{ 
					ab--;
					turn_taken=1;
	   		        int j =-1, sign;
    	           	l = encoder_value();
					sign = (direction<=2) ? 1 : (-1);
					if( direction == 1 || direction == 3)   //north direction
    					y2 = y2 + (sign)*l;     //changing y-coordinate
    				else 
     					x2 = x2 + (sign)*l;     //changing x-coordinate
					all_cordinate[all_cordinate_index][0] = x2;
					all_cordinate[all_cordinate_index][1] = y2;
		   			all_cordinate_index+=1;	
					if(x2==x1 && y2==y1)
						return z;
					break;
				}
	
 				else if(turn_stored[ab]==40)
				{ 
					ab--;
					turn_taken=1;
   					int j =-1, sign;
    				l = encoder_value();
					sign = (direction<=2) ? 1 : (-1);
					if( direction == 1 || direction == 3)   //north direction
   	 					y2 = y2 + (sign)*l;     //changing y-coordinate
    				else 
     					x2 = x2 + (sign)*l;     //changing x-coordinate
					all_cordinate[all_cordinate_index][0] = x2;
						all_cordinate[all_cordinate_index][1] = y2;
				   	all_cordinate_index+=1;	
					if(x2==x1 && y2==y1)
						return z;
					turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed, 0); 
					turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed, 0); 
					direction = (direction>2)?(direction-2):(direction+2);	
					break;	
				}
			
			}	
			else
  				move(clientID,leftMotorHandle,rightMotorHandle, turn_speed ,turn_speed);
  		}
	}//if ends node check vala
	*/
	
		
		if(sensor_values[0]>0.6 && sensor_values[1]<0.6 && sensor_values[2]<0.6 && sensor_values[3]>0.6)			//loop for detecting dead end 
		{
			while(simxGetConnectionId(clientID)!=-1)
        	{   //reading sensors
				sensor_values = read_sensors(clientID, sensor);
				printf("deadend ke andar checking if aage end aaya kya \n");
        		if(sensor_values[0]>0.5 && sensor_values[1]>0.2 && sensor_values[2]>0.2 && sensor_values[3]>0.5)
        		{
					while(simxGetConnectionId(clientID)!=-1)
					{
						sensor_values = read_sensors(clientID, sensor);
        				if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
						{	//straight line
							int j =-1, sign;
							l = encoder_value();
							sign = (direction<=2) ? 1 : (-1);
							if( direction == 1 || direction == 3)   //north direction
    							y2 = y2 + (sign)*l;     //changing y-coordinate
   		     				else 
     							x2 = x2 + (sign)*l;     //changing x-coordinate
     						all_cordinate[all_cordinate_index][0] = x2;
     						all_cordinate[all_cordinate_index][1] = y2;
     						all_cordinate_index+=1;
							direction = (direction>2)?(direction-2):(direction+2);
															
							check_all[abc] = 40;
							turn_taken=1;
							abc++;
						
							ab--;
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
				
				
   		else if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4 && sensor_values[4]<0.4)							//end of maze
		{
			while(simxGetConnectionId(clientID)!=-1)
       		{   //reading sensors
				sensor_values = read_sensors(clientID, sensor);
							
           		if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
				{	//straight line
					int j =-1, sign;
					 l = encoder_value();
					sign = (direction<=2) ? 1 : (-1);
					if( direction == 1 || direction == 3)   //north direction
						y = y + (sign)*l;     //changing y-coordinate
					 else 
						x = x + (sign)*l;     //changing x-coordinate
					all_cordinate[all_cordinate_index][0] = x;
					all_cordinate[all_cordinate_index][1] = y;
					all_cordinate_index+=1;
					turn_taken=1;
     				direction = (direction>2)?(direction-2) : direction+2;
					printf("ByE bYe u-turn\n");
					break;
				}
				else
				{	
					printf("end of maze aaya so left turn\n");
                   	move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns 180 left turn
				}
			}
		}
			
		else if(sensor_values[3]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[0]<0.3)					//for junction or t-shape
		{
				while(simxGetConnectionId(clientID)!=-1)
            	{   //reading sensors
					sensor_values = read_sensors(clientID, sensor);

					if(sensor_values[0]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[3]<0.3)
					{	printf("j/t khtm hone tak straight\n");
                    	move(clientID, leftMotorHandle, rightMotorHandle, turn_speed, turn_speed);
					}
					else
					{
						int j =-1, sign;
						l = encoder_value();
						sign = (direction<=2) ? 1 : (-1);
						if( direction == 1 || direction == 3)   //north direction
    						y2 = y2 + (sign)*l;     //changing y-coordinate
   						 else 
     						x2 = x2 + (sign)*l;     //changing x-coordinate
     					all_cordinate[all_cordinate_index][0] = x2;
     					all_cordinate[all_cordinate_index][1] = y2;
     					all_cordinate_index+=1;
     					if(x2==x1 && y2==y1)
     						return z;
     						
						//if((sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)	|| (sensor_values[0]>0.7 && sensor_values[1]<0.7 && sensor_values[2]<0.7 && sensor_values[3]>0.7))	
						//{
				
							if(turn_stored[ab] == 10)
							{
								turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed, 0);
								direction = (direction==4)?1:(direction+1);
								check_all[abc] = 30;
								turn_taken=1;
								abc++;
					
								ab--;
								break;
							}
							
							else if(turn_stored[ab] == 30)
							{
								turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed, 0);
								direction = (direction==1)?4:(direction-1);
								check_all[abc] = 10;
								turn_taken=1;
								abc++;
					
								ab--;
								break;
							}	
							
							else if(turn_stored[ab] == 20)
							{
								check_all[abc] = 20;
								turn_taken=1;
								abc++;
					
								ab--;
								break;
							}						
       				//}
       			}
       		}
		}
		
		else if(sensor_values[3]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[0]>0.7)			//pure right or straight right turn detected
		{        
			while(simxGetConnectionId(clientID)!=-1)
		    {   //reading sensors
				sensor_values = read_sensors(clientID, sensor);
	
				if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4)
				{	printf("r/sr khtm hone tak straight\n");
                   	pid_correction(sensor_values,3);
                }
        		else
        		{ 	 
        			int j =-1, sign;
					l = encoder_value();
					sign = (direction<=2) ? 1 : (-1);
					if( direction == 1 || direction == 3)   //north direction
    					y2 = y2 + (sign)*l;     //changing y-coordinate
   					 else 
     					x2 = x2 + (sign)*l;     //changing x-coordinate
     				all_cordinate[all_cordinate_index][0] = x2;
     				all_cordinate[all_cordinate_index][1] = y2;
     				all_cordinate_index+=1;
     				if(x2==x1 && y2==y1)
     					return z;               		
        		    if((sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7) || (sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7))
					{
						if(turn_stored[ab] == 10)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed, 0);
							direction = (direction==4)?1:(direction+1);
							check_all[abc] = 30;
							turn_taken=1;
							abc++;
					
							ab--;
							break;
						}
		
						else if(turn_stored[ab] == 20)
						{
							check_all[abc] = 20;
							turn_taken=1;
							abc++;
					
							ab--;
							break;
						}								
       			 	}					
       			}
        	}
		}
       			
       			
	}
}
 												
   												
int pid_control(float sensor_values[])
{
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

        		right_pwm = constrain((yaw_correction), MIN_PWM, MAX_PWM);
        		left_pwm = constrain((-yaw_correction), MIN_PWM, MAX_PWM);
                       
        	    //Extra yaw correction during turns
        		if(yaw_error>0.05)
        		{
        			right_pwm-=0.25;
        		    left_pwm+=0.25;   
        		}
        		else if(yaw_error<0.05)
        		{
        		    left_pwm-=0.25;
        		    right_pwm+=0.25;
        		}
                
        		move(clientID,leftMotorHandle,rightMotorHandle,left_pwm,right_pwm);
} 												
   												
   												
int pid_correction(float sensor_values[], int index)
{
	if(index==0)
		sensor_values[0] = 0.8;
	else if(index==3)
		sensor_values[3] = 0.8;
	/*else if(index==2)
	{
		sensor_values[0] = 0.8;
		sensor_values[3] = 0.8;
	}*/
	pid_control(sensor_values);
} 												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
   												
