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
int type[100]={1};
int init_direction[100]={1};	
int path_type[100][3];			// 1st column-> left path  2nd column->straight path  3rd column->right path
int explore[100]={1};
int cordinate[100][2];
int i=-1;
int x=0,y=0,total_points=0;		//x,y -> coordinate-distance
int direction = 1;
int expo;

//for bot's motion func
float turn_speed=1;
int leftMotorHandle;
int rightMotorHandle;
int clientID;  
float *read_sensors(int clientID, int sensor[]);
int encoder_value();
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float left_pwm,float right_pwm);
float constrain(float x, float lower_limit, float higher_limit);
int sensor[5];

//in same file
int junc_check();
int old_node(int i, int expo, int turn_stored[],int t);
int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed);
int find(int explore[],int type[],int cordinate[][2],int k,int total_points);		
int travel(int z,int cordinate[][2],int turn_stored[],int t, int d, int expo); 	



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
		if(sensor_values[0]>0 && sensor_values[1]>0 && sensor_values[2]>0 && sensor_values[3]>0)	//all the sensor readings should be greater than 0
		{
			if(sensor_values[0]<0.7 && sensor_values[1]<0.7 && sensor_values[2]<0.7 && sensor_values[3]<0.7 && sensor_values[4]<0.7)							//end of maze
			{
				while(simxGetConnectionId(clientID)!=-1)
				{	//reading sensors
					sensor_values = read_sensors(clientID, sensor);
					if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)
					{	//straight line
						int j =-1, sign;
						sign = (direction<=2) ? (1) : (-1);
						if(direction==1 || direction==3)	//north direction
							y = y + (sign)*l;	//changing y-coordinate
						else
							x = x + (sign)*l;	//changing x-coordinate
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
				if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)	//pure left or straight left turn
				{
					while(simxGetConnectionId(clientID)!=-1)
					{	//reading sensor values
						sensor_values = read_sensors(clientID, sensor);
						if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)	//getting till end of node
							pid_correction(sensor_values,0);
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
									{	//reading sensors
											sensor_values = read_sensors(clientID, sensor);
											if(sensor_values[0] > 0.5 && sensor_values[1] < 0.5 && sensor_values[2] < 0.5 && sensor_values[3] > 0.5 )
												break;
											else
												move(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed);
										}
										direction = (direction==1)?4:(direction-1);
										turn_stored[t] = 10;
										t++;
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
									turn_stored[t] = 10;
									t++;
									node_detected = 1;
									break;
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
									node_detected = 1;
									unexplored_turn = old_node(i,100,turn_stored,t);
								if(unexplored_turn != 20 && unexplored_turn != -2)
									{
										turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed);
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
									else if (unexplored_turn == 20)
									{
										turn_stored[t] = 20;
										t++;
									}
									break;
								}
								else			//new node
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
									turn_stored[t] = 10;
									t++;
									node_detected = 1;
									break;
								 }
								break;
							}
						}
					}
				}

				else if(sensor_values[3]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.5 && sensor_values[0]>0.7)			//pure right or straight right turn detected
				{
						while(simxGetConnectionId(clientID)!=-1)
						{	//reading sensors
							sensor_values = read_sensors(clientID, sensor);
							if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4)
								pid_correction(sensor_values,3);
							else
							{
								if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)		//pure right
								{
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
										turn_stored[t] = 30;
										t++;
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
										turn_stored[t] = 30;
										t++;
									}
									node_detected = 1;
									break;
								}

								else if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)		//straight-right node
								{
									//finding coordinate points of the junction
									l = encoder_value();
									i = junc_check(l);
									if(i != -1)	//old node
									{
										unexplored_turn = old_node(i, 200,turn_stored,t);
										node_detected = 1;
										if(unexplored_turn != 20 && unexplored_turn != -2)
										{
											turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed);
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
										turn_stored[t] = 20;
										t++;
										node_detected = 1;
									}
									break;
								 }
							}
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
							if(sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7)		//t-shape	
							{
								//finding coordinate points of the junction
								l = encoder_value();
								i = junc_check(l);
								if(i != -1)		//old node
								{
									unexplored_turn = old_node(i, 300,turn_stored,t);
									if(unexplored_turn != 20 && unexplored_turn != -2)
									{
										turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed);
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
									unexplored_turn = old_node(i, 400,turn_stored,t);
									if (unexplored_turn != 20 && unexplored_turn != -2)
									{
										turn(clientID, leftMotorHandle, rightMotorHandle, unexplored_turn*turn_speed,-1*unexplored_turn*turn_speed);
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
								else	//new node
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
	printf("t= %d\n", t);
	for(i = 0;i<t;i++)
	{
		printf("turns stored = %d \n", turn_stored[i]);
	}
	printf("Simulation Stopped!");
}



float *read_sensors(int clientID, int sensor[])
{
	static float sensor_value[5];
	float* auxValues = NULL;
	int* auxValuesCount = NULL;
	simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[0] = auxValues[10];
	simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[1] = auxValues[10];
	simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[2] = auxValues[10];
	simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[3] = auxValues[10];
	simxReadVisionSensor(clientID, sensor[4], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[4], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[4] = auxValues[10];
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
	if( direction == 1 || direction == 3)
		y = y + (sign)*l;
	else
		x = x + (sign)*l;
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

										   
int old_node(int i, int expo, int turn_stored[], int t)
{
	int repeat = 1, unexplored_turn=-2,target=0;
	while(repeat==1)
	{
		int dl=-1,dr=-1,dir;
		dir = init_direction[i];	//initial direction of bot on that node
		dl = (dir==1)? 4 : (dir-1);

		//for getting the path from where the bot has come
		if((direction - dir) == 2 || (direction-dir)==(-2))
		{
			//bot has explore the straight path
			if (path_type[i][1] == -1)
				explore[i]+=1;
			path_type[i][1] = 1;
		}	
		else if(direction == dl)
		{
			//bot has explore the right path
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
				target = (dir==4)?1:(dir+1);	//where bot has to go
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
			//finding nearest node                  
			int z;
			z= find( explore, type,cordinate, i, total_points); //block for finding cordinate;
			if(z == -1)
			{
				unexplored_turn = -3;
				move(clientID, leftMotorHandle, rightMotorHandle,0,0);
				break;
			}
			i = travel( z, cordinate, turn_stored, t, i, expo);
			x = cordinate[i][0];
			y = cordinate[i][1];
			repeat=1;
		}
	}
	return  unexplored_turn;
}


int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed)
{
	int v=0, count=0;
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
				
						
int travel(int z,int cordinate[][2],int turn_stored[],int t, int d, int expo)	//function for retracing
{
	float *sensor_values;
	float turn_speed=1;
	int v,x2,y2,x1,y1,c =0 ,count = 0,ab = 0,l;
	x1=cordinate[z][0];
	y1=cordinate[z][1];
	y2=cordinate[d][1];
	x2=cordinate[d][0];

	if(expo==100)
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

	if(expo==200)
	{
		int bb=0;
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

	if(expo==300 || expo==400)
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
				if(expo == 400)
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
		if(sensor_values[0]>0.6 && sensor_values[1]<0.6 && sensor_values[2]<0.6 && sensor_values[3]>0.6)			//loop for detecting dead end 
		{
			while(simxGetConnectionId(clientID)!=-1)
			{
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
						sensor_values = read_sensors(clientID, sensor);
						if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)
						{
							int j =-1, sign;
							l = encoder_value();
							sign = (direction<=2) ? 1 : (-1);
							if( direction == 1 || direction == 3)
								y2 = y2 + (sign)*l;
							else
								x2 = x2 + (sign)*l;
							direction = (direction>2)?(direction-2):(direction+2);
							turn_taken=1;
							ab--;
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

		else if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4 && sensor_values[4]<0.4)	//end of maze
		{
			while(simxGetConnectionId(clientID)!=-1)
			{	//reading sensors
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
						if( direction == 1 || direction == 3)   //north direction
							y2 = y2 + (sign)*l;     //changing y-coordinate
						else 
							x2 = x2 + (sign)*l;     //changing x-coordinate
						if(x2==x1 && y2==y1)
							return z;
	
						if(turn_stored[ab] == 10)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
							direction = (direction==4)?1:(direction+1);
							turn_taken=1;
							ab--;
							break;
						}

						else if(turn_stored[ab] == 30)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed);
							direction = (direction==1)?4:(direction-1);
							turn_taken=1;
							ab--;
							break;
						}	
	
						else if(turn_stored[ab] == 20)
						{
							turn_taken=1;
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
			{	//reading sensors
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
						if( direction == 1 || direction == 3)   //north direction
							y2 = y2 + (sign)*l;     //changing y-coordinate
						else 
							x2 = x2 + (sign)*l;     //changing x-coordinate
						if(x2==x1 && y2==y1)
							return z;       

						if(turn_stored[ab] == 10)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
							direction = (direction==4)?1:(direction+1);
							turn_taken=1;
							ab--;
							break;
						}

						else if(turn_stored[ab] == 20)
						{
							turn_taken=1;
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
			{	//reading sensor values
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
							y2 = y2 + (sign)*l;     //changing y-coordinate
						else 
							x2 = x2 + (sign)*l;     //changing x-coordinate
						if(x2==x1 && y2==y1)
							return z;

						if(turn_stored[ab] == 30)
						{
							turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed);
							direction = (direction==1)?4:(direction-1);
							turn_taken=1;
							ab--;
							break;
						}	

						else if(turn_stored[ab] == 20)
						{
							turn_taken=1;
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
		sensor_values[0] = 0.76;
	else if(index==3)
		sensor_values[3] = 0.76;
	pid_control(sensor_values);
}
