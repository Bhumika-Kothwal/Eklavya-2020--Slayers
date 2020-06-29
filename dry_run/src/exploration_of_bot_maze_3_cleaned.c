#include "vrep.h"

//C Headers
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>


#define MAX_PWM 10
#define MIN_PWM 3

//Line Following Tuning Parameters
float yaw_kP= 0.69;
float yaw_kI= 0;
float yaw_kD= 0.17;

//FOR LINE FOLLOWING
float yaw_error=0, yaw_prev_error=0, yaw_difference=0, yaw_cumulative_error=0, yaw_correction=0;
int weights[4] = {3,1,-1,-3};
float left_pwm = 0, right_pwm = 0;
int pid_control(float sensor_values[]);
int pid_control_slow(float sensor_values[]);
int pid_correction(float sensor_values[], int index);

//node properties
int i=-1;
int cordinate[100][2];
int x=0,y=0,total_points=0;		//x,y -> coordinate-distance
int type[100]={1};
int explore[100]={1};
int init_direction[100]={1};
int direction = 1;		// 1->North		2->East		3->South	4->West 
int path_type[100][3];			// 1st column->left path  2nd column->straight path  3rd column->right path

//for plotting og graph
int array[100];
int array_index=0;
int cordinate_index=0;
int endx;
int endy;
int graph[100][100];

//for bot's motion func
float turn_speed=1.5;

//for retracing of path
int node_type;		// 100->straight left	200->straight right		300->t-shape	400->junction
int turn_stored[300];
int turn_stored_index;
int turns_for_retracing[300];
int turns_for_retracing_index;

//bot handles
int leftMotorHandle;
int rightMotorHandle;
int clientID;  
int sensor[5];

//for dikshtra's algo
int endofmazecordinate ;
int path[100];
int updated[100][3], by[100];

//function defined
float *read_sensors(int clientID, int sensor[]);
int encoder_value();
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float left_pwm,float right_pwm);
float constrain(float x, float lower_limit, float higher_limit);
int findnow(int x , int y , int tp);
int junc_check();
int old_node(int i, int node_type, int turn_stored[],int turn_stored_index);
int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed);
int find(int explore[],int type[],int cordinate[][2],int k,int total_points);
int travel(int z,int cordinate[][2], int d, int node_type);
void plotgraph();
void shortest_path();


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
	int l;
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
			
			if(sensor_values[0]>0 && sensor_values[1]>0 && sensor_values[2]>0 && sensor_values[3]>0)					
			{	//all the sensor readings should be greater than 0
				
				if(sensor_values[0]<0.7 && sensor_values[1]<0.7 && sensor_values[2]<0.7 && sensor_values[3]<0.7 && sensor_values[4]<0.7)							
				{	//end of maze detected
					
					while(simxGetConnectionId(clientID)!=-1)
					{	//reading sensors
						sensor_values = read_sensors(clientID, sensor);
						if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
						{	//straight line
							int j =-1, sign;
							sign = (direction<=2) ? (1) : (-1);
							if( direction == 1 || direction == 3)	//north direction
								y = y + (sign)*l;	//changing y-coordinate
							else
								x = x + (sign)*l;	//changing x-coordinate
							i=total_points+1;
							cordinate[i][0]=x;
							cordinate[i][1]=y;
							explore[i] = 5;
							type[i] = 4;
							total_points+=1;
							endofmazecordinate =i;
							endx=x;
							endy=y;
							cordinate_index = findnow(x , y , total_points);
							array[array_index]=cordinate_index;
							array_index++;
							direction = (direction>2)?(direction-2) : direction+2;
							node_detected = 1;
							break;
						}
						else
							move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns 180 left turn
					}
				}
			
				else
				{
					if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
					{	//pure left or straight left turn

						while(simxGetConnectionId(clientID)!=-1)
						{	//reading sensor values
							sensor_values = read_sensors(clientID, sensor);

							if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)	//getting till end of node
								pid_correction(sensor_values,0);
							else
							{
								if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)			
								{	//pure left

									l = encoder_value();
									i = junc_check(l);
									if(i != -1)		//old node
									{	//find the nearest node
										while(simxGetConnectionId(clientID)!=-1)
											{	//reading sensors
												sensor_values = read_sensors(clientID, sensor);
												if(sensor_values[0] > 0.5 && sensor_values[1] < 0.5 && sensor_values[2] < 0.5 && sensor_values[3] > 0.5 )
													break;
												else
													move(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed);
											}
											direction = (direction==1)?4:(direction-1);
											turn_stored[turn_stored_index] = 10; 
											turn_stored_index++;
											node_detected = 1;
											break;
									}		
									else	//new node
									{
										i=total_points+1;
										cordinate[i][0]=x;
										cordinate[i][1]=y;
										type[i]=1;
										total_points++;
										path_type[i][0] = 1;
										init_direction[i] = direction;	
										explore[i] = 1;
										turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed);
										direction = (direction==1) ? 4 : (direction-1);
										turn_stored[turn_stored_index] = 10;
										turn_stored_index++;
										node_detected = 1;
										break;
									}
									break;
								}

								else if(sensor_values[0]>0.7 && sensor_values[1]<0.5 && sensor_values[2]<0.5 && sensor_values[3]>0.7)				
								{	//straight left

									l = encoder_value();
									i = junc_check(l);
									if(i != -1)			//old node	
									{
										node_detected = 1;
										unexplored_turn = old_node(i,100,turn_stored,turn_stored_index);
										if(unexplored_turn != 20 && unexplored_turn != -2)
										{
											turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed);
											if(unexplored_turn < 0 )
											{	direction = (direction==1) ? 4 : (direction-1);
												turn_stored[turn_stored_index] = 10;
												turn_stored_index++;
											}
											else
											{	direction = (direction==4) ?1 : (direction+1);
												turn_stored[turn_stored_index] = 30;
												turn_stored_index++;
											}
										
										}
										else if (unexplored_turn == 20)
										{
											turn_stored[turn_stored_index] = 20;
											turn_stored_index++;
										}
										break;
									}
									else	//new node
									{
										i=total_points+1;
										cordinate[i][0]=x;
										cordinate[i][1]=y;
										type[i]=2;
										total_points++;
										path_type[i][0] = 1;
										path_type[i][1] = -1;
										explore[i] = 1;
										init_direction[i] = direction;							
										turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed);
										direction = (direction==1) ? 4 : (direction-1);
										turn_stored[turn_stored_index] = 10;
										turn_stored_index++;
										node_detected = 1;
										break;
									}
									break;
								}
							}
						}
					}
				
				
					else if(sensor_values[3]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.5 && sensor_values[0]>0.7)
					{	//pure right or straight right turn detected

						while(simxGetConnectionId(clientID)!=-1)
						{   //reading sensors
							sensor_values = read_sensors(clientID, sensor);
		
							if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4)	//getting till end of node
								pid_correction(sensor_values,3);
							else
							{
								if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)		
								{	//pure right

									l = encoder_value();
									i = junc_check(l);
									if(i != -1)		//old node
									{	//finding nearest node
										while(simxGetConnectionId(clientID)!=-1)
										{	//reading sensors
											sensor_values = read_sensors(clientID, sensor);
											if(sensor_values[0] > 0.7 && sensor_values[1] < 0.3 && sensor_values[2] < 0.3 && sensor_values[3] > 0.7 )
												break;
											else
												move(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
										}
										direction = (direction==4)?1:(direction+1);
										turn_stored[turn_stored_index] = 30;
										  
										turn_stored_index++;
									}
									else	//new node
									{
										i=total_points+1;
										cordinate[i][0]=x;
										cordinate[i][1]=y;
										type[i]=1;
										total_points++;
										path_type[i][2] = 1;
										init_direction[i] = direction;
										explore[i] = 1;
										turn(clientID, leftMotorHandle, rightMotorHandle, turn_speed, -1*turn_speed);
										direction = (direction==4) ?1 : (direction+1);
										turn_stored[turn_stored_index] = 30;
										turn_stored_index++;
									}
									node_detected = 1;
									break;
								}

								else if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)
								{	//straight-right node
									
									l = encoder_value();
									i = junc_check(l);
									if(i != -1)		//old node
									{
										unexplored_turn = old_node(i, 200,turn_stored,turn_stored_index);
										node_detected = 1;
										if(unexplored_turn != 20 && unexplored_turn != -2)
										{
											turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed);
											if(unexplored_turn < 0 )
											{	direction = (direction==1) ? 4 : (direction-1);
												turn_stored[turn_stored_index] = 10;
												turn_stored_index++;
											}
											else
											{	direction = (direction==4) ?1 : (direction+1);
												turn_stored[turn_stored_index] = 30;
												  
												turn_stored_index++;
											}
										}
										if(unexplored_turn == 20)
										{
											turn_stored[turn_stored_index] = 20;
											turn_stored_index++;
										}
									}
									else	//new node
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
										turn_stored[turn_stored_index] = 20;
										turn_stored_index++;
										node_detected = 1;
									}
									break;
								}
							}
						}	
					}


					else if(sensor_values[3]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[0]<0.3)
					{	//junction or t-shape

						while(simxGetConnectionId(clientID)!=-1)
						{	//reading sensors
							sensor_values = read_sensors(clientID, sensor);

							if(sensor_values[0]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[3]<0.3)	//getting till the end of node
								move(clientID, leftMotorHandle, rightMotorHandle, turn_speed, turn_speed);
							else
							{
								if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)
								{	//t-shape

									l = encoder_value();
									i = junc_check(l);
									if(i != -1)		//old node
									{
										unexplored_turn = old_node(i, 300,turn_stored,turn_stored_index);
										if(unexplored_turn != 20 && unexplored_turn != -2)
										{
											turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed);
											if(unexplored_turn < 0 )
											{	
												direction = (direction==1) ? 4 : (direction-1);
												turn_stored[turn_stored_index] = 10;
												turn_stored_index++;
											}
											else
											{
												direction = (direction==4) ?1 : (direction+1);
												turn_stored[turn_stored_index] = 30;
												turn_stored_index++;
											}
										}	
										if(unexplored_turn == 20)
										{
											turn_stored[turn_stored_index] = 20;
											turn_stored_index++;
										}
										node_detected = 1;
									}
									else	//new node
									{
										i=total_points+1;
										cordinate[i][0]=x;
										cordinate[i][1]=y;
										type[i]=2;
										total_points++;
										path_type[i][0] = 1;
										path_type[i][2] = -1;
										explore[i] = 1;
										init_direction[i] = direction;								
										turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed);
										direction = (direction==1) ? 4 : (direction-1);
										turn_stored[turn_stored_index] = 10;
										node_detected = 1;
										turn_stored_index++;
									}
									break;
								}

								else if(sensor_values[0]>0.7 && sensor_values[1]<0.7 && sensor_values[2]<0.7 && sensor_values[3]>0.7)
								{	//junction

									l = encoder_value();
									i = junc_check(l);
									if(i != -1)		//old node
									{
										unexplored_turn = old_node(i, 400,turn_stored,turn_stored_index);
										if (unexplored_turn != 20 && unexplored_turn != -2)
										{
											turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed);
											if(unexplored_turn < 0 )
											{	direction = (direction==1) ? 4 : (direction-1);
												turn_stored[turn_stored_index] = 10;
												turn_stored_index++;
											}
											else
											{	direction = (direction==4) ?1 : (direction+1);
												turn_stored[turn_stored_index] = 30;
												turn_stored_index++;
											}
										}	//turns left
										if(unexplored_turn == 20)
										{	
											turn_stored[turn_stored_index] = 20;
											turn_stored_index++;
										}
										node_detected = 1;
									}
									else		//new node
									{
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
										turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed, turn_speed);
										direction = (direction==1) ? 4 : (direction-1);
										turn_stored[turn_stored_index] = 10;
										turn_stored_index++;
										node_detected = 1;
									}
									break;
								}
							}
						}
					}


					else if(sensor_values[0]>0.6 && sensor_values[1]<0.6 && sensor_values[2]<0.6 && sensor_values[3]>0.6)
					{	//dead end 
						while(simxGetConnectionId(clientID)!=-1)
						{	//reading sensors
							sensor_values = read_sensors(clientID, sensor);
							int count=0;
							if(sensor_values[0]>0.5 && sensor_values[1]>0.2 && sensor_values[2]>0.2 && sensor_values[3]>0.5)
							{
							
								while(count<=600000)
								{
									move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);
									count++;
								}
								while(simxGetConnectionId(clientID)!=-1)
								{
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
										direction = (direction>2)?(direction-2):(direction+2);
										node_detected = 1;
										break;
									}
									else
										move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns 180 left turn
								}
								break;
							}
							else
								break;
						}
					}

					if(unexplored_turn == -3)
					{
						move(clientID, leftMotorHandle, rightMotorHandle, 0, 0);
						break;
					}
					

					//straight line
					pid_control(sensor_values);
					if(node_detected == 1)
					{
						int count_1=0;
						while(count_1<=5000)
						{
							sensor_values = read_sensors(clientID, sensor);
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

	for(i = 0;i<=total_points;i++)
	{
		printf("co-ordinate point: %d,%d  explore_paths: %d   type_path: %d initial_direction: %d  path_type: %d %d %d\n",cordinate[i][0],cordinate[i][1],explore[i],type[i],init_direction[i], path_type[i][0], path_type[i][1], path_type[i][2]);
	}
	printf("final direction:%d \n", direction);
	printf("t= %d\n", turn_stored_index);
	for(i = 0;i<turn_stored_index;i++)
	{
		printf("turns stored = %d \n", turn_stored[i]);
	}

	for(int p = 0;p<turns_for_retracing_index;p++)
	{
		printf("retrasing values= %d \n ", turns_for_retracing[p]);
	}
	for(int w = 0 ; w<array_index ; w++)
	{
		printf("FINAL ARRAY CONNECTION = %d \n ", array[w]);
	}
	printf("END MAZE CORDINATES x= %d  y=%d \n ", endx , endy);
	plotgraph();
	
	//plotting graph;
	for(int o=1 ; o<=total_points+1 ; o++)
	{
	for(int w = 1 ; w<=total_points+1 ; w++)
	{
		printf(" %d  ", graph[w][o]);
	}
	printf("\n");
	}
	shortest_path();

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
	printf("the sensor values: %f %f %f %f %f  direction=%d\n",sensor_value[4], sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3],direction);
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
	if( direction == 1 || direction == 3)	//north direction
		y = y + (sign)*l;	//changing y-coordinate
	else
		x = x + (sign)*l;	//changing x-coordinate

	for( int f=1;f<=total_points;f++)
	{
		if(x== cordinate[f][0] && y== cordinate[f][1])
		{
			j = f;
			break;
		}
	}
	if(j!=-1)
	{
		array[array_index]=j;
		array_index++;
	}
	else
	{
		array[array_index]=total_points+1;
		array_index++;
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
int old_node(int i, int node_type, int turn_stored[], int turn_stored_index)
{
	int repeat = 1, unexplored_turn=-2,target=0;
	while(repeat==1)
	{
		int dl=-1,dr=-1,dir;
		dir = init_direction[i];		//initial direction of bot on that node
		dl = (dir==1)? 4 : (dir-1);

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
		repeat=0;
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
		{	//finding nearest node
			int z;
			z= find( explore, type, cordinate, i , total_points); //block for finding cordinate;
			if(z == -1)
			{
				unexplored_turn = -3;
				move(clientID, leftMotorHandle, rightMotorHandle,0,0);
				break;
			}
			i = travel( z, cordinate, i, node_type);
			x = cordinate[i][0];
			y = cordinate[i][1];
			repeat=1;
		}
	}
	return  unexplored_turn;
}


int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed)
{
	int count = 0;
	float *sensor_values;
	while(count<=20)
	{
		move(clientID,leftMotorHandle,rightMotorHandle,0.01,0.01);   	
		count++;
	}
	while(count<=600000)
	{
		move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,rightMotorSpeed);
		count++;
	}	
	while(simxGetConnectionId(clientID)!=-1)
	{
		sensor_values = read_sensors(clientID, sensor);
		if(sensor_values[0]>0.5 &&sensor_values[1]<0.5 &&sensor_values[2]<0.5 &&sensor_values[3]>0.5)
			break;
		else
			move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,rightMotorSpeed);
	}
}


int find(int explore[],int type[],int cordinate[][2],int k,int total_points)
{
	int x,y,x1,y1,min,final=-1;
	x=cordinate[k][0];
	y=cordinate[k][1];
	min=1000000;
	
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


int travel(int target_cordinate,int cordinate[][2],int d, int node_type)	//function for retracing
{
	float *sensor_values;
	int v;
	float turn_speed=1.5;
	int speed=0;
	int present_cordinate_x,present_cordinate_y,target_cordinate_x,target_cordinate_y,c = 0,count =0 ,l;
	int ab=0;
	target_cordinate_x=cordinate[target_cordinate][0];
	target_cordinate_y=cordinate[target_cordinate][1];
	present_cordinate_y=cordinate[d][1];		// d-> index of coordinate in coordinate array
	present_cordinate_x=cordinate[d][0];
	
	if(node_type==100)
	{
		while(count<=1000000)
		{
			move(clientID,leftMotorHandle,rightMotorHandle, turn_speed ,-1*turn_speed);
			count++;
		}
		while(simxGetConnectionId(clientID)!=-1)
		{
			sensor_values = read_sensors(clientID, sensor);
			if(sensor_values[0]>0.4&&sensor_values[1]<0.4&&sensor_values[2]<0.4&&sensor_values[3]>0.4)
			{
				direction = (direction>2)?(direction-2):(direction+2);
				break;
			}
			else
				move(clientID,leftMotorHandle,rightMotorHandle, turn_speed ,-1*turn_speed);  
		}
	}

	if(node_type==200)
	{
		while(count<=1000000)
		{
			move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed ,turn_speed);
			count++;
		}
		while(simxGetConnectionId(clientID)!=-1)
		{
			sensor_values = read_sensors(clientID, sensor);
			if(sensor_values[0]>0.4&&sensor_values[1]<0.4&&sensor_values[2]<0.4&&sensor_values[3]>0.4)
			{
				direction = (direction>2)?(direction-2):(direction+2);
				break;
			}
			else
				move(clientID,leftMotorHandle,rightMotorHandle,-1* turn_speed ,turn_speed);  
		}
	}

	if(node_type==300 || node_type==400)
	{
		while(count<=1000000)
		{
			move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed ,turn_speed);
			count++;
		}
		
		while(simxGetConnectionId(clientID)!=-1)
		{
			sensor_values = read_sensors(clientID, sensor);
			if(sensor_values[0]>0.4&&sensor_values[1]<0.4&&sensor_values[2]<0.4&&sensor_values[3]>0.4)
			{
				if(node_type == 400)
				{
					c++;
					if(c==2)
					{
						c=0;
						direction = (direction>2)?(direction-2):(direction+2);
						break;
					}
				}
				else
				{
					direction = (direction>2)?(direction-2):(direction+2);
					break;
			}
			}
			else
				move(clientID,leftMotorHandle,rightMotorHandle, -1*turn_speed ,turn_speed);
		}
	}
	turn_stored[turn_stored_index] = 40;
	turn_stored_index++;
	ab = turn_stored_index-2;
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

		if(sensor_values[0]>0.6 && sensor_values[1]<0.6 && sensor_values[2]<0.6 && sensor_values[3]>0.6)			//loop for detecting dead end 
		{
			while(simxGetConnectionId(clientID)!=-1)
			{	//reading sensors
				printf("dead end ke andar checking if aage end aaya kya \n");
				sensor_values = read_sensors(clientID, sensor);
				if(sensor_values[0]>0.5 && sensor_values[1]>0.2 && sensor_values[2]>0.2 && sensor_values[3]>0.5)
				{
					count = 0;
					while(count<=600000)
					{
						move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);
						count++;
					}
					while(simxGetConnectionId(clientID)!=-1)
					{
						printf("u-turn le rha h \n");
						sensor_values = read_sensors(clientID, sensor);
						if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
						{	//straight line
							int j =-1, sign;
							l = encoder_value();
							sign = (direction<=2) ? 1 : (-1);
							if( direction == 1 || direction == 3)   //north direction
								present_cordinate_y = present_cordinate_y + (sign)*l;     //changing y-coordinate
							else
								present_cordinate_x = present_cordinate_x + (sign)*l;     //changing x-coordinate
							
							direction = (direction>2)?(direction-2):(direction+2);
							turns_for_retracing[turns_for_retracing_index] = 40;
							turn_taken=1;
							turns_for_retracing_index++;
							break;
						}
						else
							move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns 180 left turn
					}
					break;
				}
				else
					break;
			}
		}


		else if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4 && sensor_values[4]<0.4)			//end of maze
		{
			while(simxGetConnectionId(clientID)!=-1)
			{	//reading sensors
				sensor_values = read_sensors(clientID, sensor);

				if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			
				{	//straight line
					present_cordinate_x = endx;
					present_cordinate_y = endy;
					turn_taken=1;
					direction = (direction>2)?(direction-2) : direction+2;
					break;
				}
				else
					move(clientID,leftMotorHandle,rightMotorHandle,-1*turn_speed,turn_speed);		//turns 180 left turn
			}
		}
			
		else if(sensor_values[3]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[0]<0.3)					//for junction or t-shape
		{
			while(simxGetConnectionId(clientID)!=-1)
			{	//reading sensors
				sensor_values = read_sensors(clientID, sensor);
				if(sensor_values[0]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[3]<0.3)
					move(clientID, leftMotorHandle, rightMotorHandle, turn_speed, turn_speed);

				else
				{
					if((sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7) || (sensor_values[0]>0.7 && sensor_values[1]<0.7 && sensor_values[2]<0.7 && sensor_values[3]>0.7))
					{
						int j =-1, sign;
						l = encoder_value();
						sign = (direction<=2) ? 1 : (-1);
						if( direction == 1 || direction == 3)	//north direction
							present_cordinate_y = present_cordinate_y + (sign)*l;		//changing y-coordinate
						 else
							present_cordinate_x = present_cordinate_x + (sign)*l;		//changing x-coordinate

						cordinate_index = findnow(present_cordinate_x , present_cordinate_y , total_points);
						array[array_index]=cordinate_index;
						array_index++;
						
						if(present_cordinate_x==target_cordinate_x && present_cordinate_y==target_cordinate_y)
							return target_cordinate;
						
						if(turn_stored[ab] == 10)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
							direction = (direction==4)?1:(direction+1);
							turn_stored[turn_stored_index] = 30;
							turn_stored_index++;
							turns_for_retracing[turns_for_retracing_index] = 30;
							turn_taken=1;
							turns_for_retracing_index++;
							ab--;
							break;
						}
						else if(turn_stored[ab] == 30)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed);
							direction = (direction==1)?4:(direction-1);
							turn_stored[turn_stored_index] = 10;
							turn_stored_index++;
							turns_for_retracing[turns_for_retracing_index] = 10;
							turn_taken=1;
							turns_for_retracing_index++;
							ab--;
							break;
						}

						else if(turn_stored[ab] == 20)
						{
							turn_stored[turn_stored_index] = 20;
							turn_stored_index++;
							turns_for_retracing[turns_for_retracing_index] = 20;
							turn_taken=1;
							turns_for_retracing_index++;
							ab--;
							break;
						}
					}
				}
			}
		}
		
		else if(sensor_values[3]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[0]>0.7)			//pure right or straight right turn detected
		{
			while(simxGetConnectionId(clientID)!=-1)
			{
				// reading sensors
				sensor_values = read_sensors(clientID, sensor);
	
				if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4)
					pid_correction(sensor_values,3);

				else
				{
					if((sensor_values[0]>0.5 && sensor_values[1]>0.2 && sensor_values[2]>0.2 && sensor_values[3]>0.5) || (sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7))
					{
						int j =-1, sign;
						l = encoder_value();
						sign = (direction<=2) ? 1 : (-1);
						if( direction == 1 || direction == 3)	//north direction
							present_cordinate_y = present_cordinate_y + (sign)*l;		//changing y-coordinate
						else
							present_cordinate_x = present_cordinate_x + (sign)*l;		//changing x-coordinate

						cordinate_index = findnow(present_cordinate_x , present_cordinate_y , total_points);
						array[array_index]=cordinate_index;
						array_index++;
							
						if(present_cordinate_x==target_cordinate_x && present_cordinate_y==target_cordinate_y)
							return target_cordinate;

						if(turn_stored[ab] == 10)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
							direction = (direction==4)?1:(direction+1);
							turn_stored[turn_stored_index] = 30;
							turn_stored_index++;
							turns_for_retracing[turns_for_retracing_index] = 30;
							turn_taken=1;
							turns_for_retracing_index++;
							ab--;
							break;
						}
						else if(turn_stored[ab] == 20)
						{
							turn_stored[turn_stored_index] = 20;
							turn_stored_index++;
							turns_for_retracing[turns_for_retracing_index] = 20;
							turn_taken=1;
							turns_for_retracing_index++;
							ab--;
							break;
						}
						else if(turn_stored[ab] == 40)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed);
							direction = (direction>2)?(direction-2):(direction+2);
							turn_stored[turn_stored_index] = 40;
							turn_stored_index++;
							turns_for_retracing[turns_for_retracing_index] = 40;
							turn_taken=1;
							turns_for_retracing_index++;
							ab--;
							break;
						}
					}
				}
			}
		}
		
		if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			//pure left or straight left turn
		{
			while(simxGetConnectionId(clientID)!=-1)
			{
				//reading sensor values
				sensor_values = read_sensors(clientID, sensor);

				if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)	//getting till end of node
					pid_correction(sensor_values,0);

				else
				{
					if((sensor_values[0]>0.5 && sensor_values[1]>0.2 && sensor_values[2]>0.2 && sensor_values[3]>0.5) || (sensor_values[0]>0.7 && sensor_values[1]<0.5 && sensor_values[2]<0.5 && sensor_values[3]>0.7))
					{
						int j =-1, sign;
						l = encoder_value();
						sign = (direction<=2) ? 1 : (-1);
						if( direction == 1 || direction == 3)   //north direction
							present_cordinate_y = present_cordinate_y + (sign)*l;     //changing y-coordinate
						else
							present_cordinate_x = present_cordinate_x + (sign)*l;     //changing x-coordinate

						cordinate_index = findnow(present_cordinate_x , present_cordinate_y , total_points);
						array[array_index]=cordinate_index;
						array_index++;
						
						if(present_cordinate_x==target_cordinate_x && present_cordinate_y==target_cordinate_y)
							return target_cordinate;
						
						if(turn_stored[ab] == 30)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed);
							direction = (direction==1)?4:(direction-1);
							turn_stored[turn_stored_index] = 10;
							turn_stored_index++;
							turns_for_retracing[turns_for_retracing_index] = 10;
							turn_taken=1;
							turns_for_retracing_index++;
							ab--;
							break;
						}
						else if(turn_stored[ab] == 20)
						{
							turn_stored[turn_stored_index] = 20;
							turn_stored_index++;
							turns_for_retracing[turns_for_retracing_index] = 20;
							turn_taken=1;
							turns_for_retracing_index++;
							ab--;
							break;
						}
						else if(turn_stored[ab] == 40)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
							direction = (direction>2)?(direction-2):(direction+2);
							turn_stored[turn_stored_index] = 40;
							turn_stored_index++;
							turns_for_retracing[turns_for_retracing_index] = 40;
							turn_taken=1;
							turns_for_retracing_index++;
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
			pos = 0.25;
		else
			pos = -0.25;
	}
	yaw_error = pos;
	//calculating_yaw_correction :-
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
		right_pwm-=0.68;
		left_pwm+=0.68;   
	}
	else if(yaw_error<0.05)
	{
		left_pwm-=0.68;
		right_pwm+=0.68;
	}
	move(clientID,leftMotorHandle,rightMotorHandle,left_pwm,right_pwm);
}


int pid_control_slow(float sensor_values[])
{
	float yaw_kP_slow= 0.5;
	float yaw_kI_slow= 0.1;
	float yaw_kD_slow= 0.36;
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
	yaw_difference = (yaw_error - yaw_prev_error);
	yaw_cumulative_error += yaw_error;
	if(yaw_cumulative_error > 0.3)
		yaw_cumulative_error = 0.3;

	else if(yaw_cumulative_error < -0.3)
		yaw_cumulative_error = -0.3;
	
	yaw_correction = yaw_kP_slow*yaw_error + yaw_kI_slow*yaw_cumulative_error + yaw_kD_slow*yaw_difference;
	yaw_prev_error = yaw_error;
	right_pwm = constrain((yaw_correction), 1, 10);
	left_pwm = constrain((-yaw_correction), 1, 10);

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
	pid_control_slow(sensor_values);
}


int findnow(int x , int y , int tp)
{
	int abs;
	for( abs = 1 ; abs<=tp ; abs++)
	{
		if(cordinate[abs][0]==x && cordinate[abs][1]==y)
		return abs;
	}
}


void plotgraph()
{
	for(int b=0;b<array_index-1;b++)
	{
		if(array[b]!=-1&&(array[b]!=array[b-1])&&(array[b]!=array[b+1])&& array[b-1] !=-1)
		{
			graph[array[b]][array[b-1]] = 1;//instead of 1 here should be distance of path
			graph[array[b-1]][array[b]] = 1;
			graph[array[b]][array[b+1]] = 1;
			graph[array[b+1]][array[b]] = 1;
		}
	}
}


void shortest_path()
{
	int distance[100], compare[100];
	distance[0]= (-1);
	distance[1] = (-1);
	//making shortest distance array
	for(int i=2; i<=total_points+1; i++)
	{
		distance[i] = (graph[1][i] == 0) ? 1000000 : graph[1][i];
		compare[i] = distance[i];
	}
	//finding shortest distance to nodes till we reach end of maze node
	int count=1, u=0, b=0;
	while(count<=total_points+1)
	{
		count++;
		int min, index_considered, check;
		min = compare[count];
		index_considered = count;
		for(int i=2; i<=total_points+1; i++)
		{
			if(min>compare[i])
			{
				min = distance[i];
				check = compare[i];
				index_considered = i;
			}
		}
		compare[index_considered] = 1000000;
		if((index_considered == endofmazecordinate) || (check == 1000000))
			break;
		int flag=0, v=0;
		for(int i = index_considered+1; i<=total_points+1; i++)
		{
			if(((graph[index_considered][i]+min) < distance[i]) && (graph[index_considered][i] >= 1))
			{
				flag = 1;
				distance[i] = graph[index_considered][i]+min;
				compare[i] = distance[i];
				by[b] = index_considered;
				updated[u][v] = i;
				v++;
			}
		}

		if(flag==1)
		{
			flag=0;
			u++;
			b++;
		}
	}
	int repeat = 1;
	int num, p=1;
	int path_cordinate_index[100];
	path_cordinate_index[0] = endofmazecordinate;
	num = endofmazecordinate;
	// storing the shortest path in form of indices of cordinate array
	for(int i=u-1; i>=0; i--)
	{
		for(int j=0;j<3;j++)
		{
			if(updated[i][j] == num)
			{
				num = by[i];
				path_cordinate_index[p] = num;
				p++;
			}
		}
	}
	int direction_of_bot = 1;
	//accessing coordinate points from indices of coordinate array stored in 'path_cordinate_index' array
	//calculating the turn to be taken and storing it
	int path_turn[100];
	int present_x, present_y, target_x, target_y, index;
	present_x = cordinate[1][0];
	present_y = cordinate[1][1];
	for(int i=p-1; i>=0; i--)
	{
		index = path_cordinate_index[i];
		target_x = cordinate[index][0];
		target_y = cordinate[index][1];
		if(present_x == target_x)
		{
			if(present_y < target_y)
			{	
				if(direction_of_bot == 1)
					path_turn[p-i-1] = 20;
				else if(direction_of_bot == 2)
					path_turn[p-i-1] = 10;
				else if(direction_of_bot == 4)
					path_turn[p-i-1] = 30;
			}
			else
			{
				if(direction_of_bot == 3)
					path_turn[p-i-1] = 20;
				else if(direction_of_bot == 2)
					path_turn[p-i-1] = 30;
				else if(direction_of_bot == 4)
					path_turn[p-i-1] = 10;
			}
		}
		else if(present_y == target_y)
		{
			if(present_x > target_x)
			{
				if(direction_of_bot == 1)
					path_turn[p-i-1] = 10;
				else if(direction_of_bot == 3)
					path_turn[p-i-1] = 30;
				else if(direction_of_bot == 4)
					path_turn[p-i-1] = 20;
			}
			else
			{
				if(direction_of_bot == 1)
					path_turn[p-i-1] = 30;
				else if(direction_of_bot == 3)
					path_turn[p-i-1] = 10;
				else if(direction_of_bot == 2)
					path_turn[p-i-1] = 20;
			}
		}
		present_x = target_x;
		present_y = target_y;
		if(path_turn[p-i-1]==10)
		{
			direction_of_bot = (direction_of_bot==1) ? 4 : (direction_of_bot-1); 
		}
		else if(path_turn[p-i-1] == 30)
		{
			direction_of_bot = (direction_of_bot==4)? 1 : (direction_of_bot+1);
		}
	}

	FILE *fpointer;
	fpointer = fopen("text_file.txt", "w");
	if(fpointer == NULL)
		printf("File couldnt be created\n");
	for(int i=0; i<=p-1; i++)
	{
		fprintf(fpointer, "%d\n", path_turn[i]);
	}
	fclose(fpointer);
}
