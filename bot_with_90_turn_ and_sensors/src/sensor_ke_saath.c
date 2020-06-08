#include "vrep.h"

// for more info refer to 
// https://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm 
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float leftMotorSpeed,float rightMotorSpeed);
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
    int n = 10;

    // save the handles
    if (argc>=n){
        portNb=atoi(argv[1]); 
        leftMotorHandle = atoi(argv[2]);
        rightMotorHandle = atoi(argv[3]);
		botbase = atoi(argv[4]);
		reference = atoi(argv[5]);
		sensor[0] = atoi(argv[6]);
		sensor[1] = atoi(argv[7]);
		sensor[2] = atoi(argv[8]);
		sensor[3] = atoi(argv[9]);
    }
    else{
        printf("Sufficient arguments not provided'!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // variables for the code
    float angle[3] = {0,0,0};
    float leftMotorSpeed = 1;
    float rightMotorSpeed = 1;
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
		printf("motion type:\n");
		scanf("%c",&s);
		while (simxGetConnectionId(clientID)!=-1){
			//getting orientation of bot with respect to reference frame
			simxGetObjectOrientation(clientID, botbase, reference, angle, simx_opmode_streaming);
			if( simxGetObjectOrientation(clientID, botbase, reference, angle, simx_opmode_buffer) == simx_return_ok)
                		printf("the angle: %f %f %f\n",angle[0],angle[1],angle[2]);
                		
            //reading sensors
			simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               // printf("sensor: %f\n", auxValues[10]);
               svalue[0] = auxValues[10];
            }
			simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               // printf("sensor: %f\n", auxValues[10]);
               svalue[1] = auxValues[10];
            }
			simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               // printf("sensor: %f\n", auxValues[10]);
               svalue[2] = auxValues[10];
            }
			simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
			if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
               // printf("sensor: %f\n", auxValues[10]);
               svalue[3] = auxValues[10];
            }
            printf("the sensor values: %f %f %f %f\n",svalue[0],svalue[1],svalue[2],svalue[3]);
            
			if(s=='f')
        		move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,rightMotorSpeed);
        	else if(s=='b')
       			move(clientID,leftMotorHandle,rightMotorHandle,-1*leftMotorSpeed,-1*rightMotorSpeed);
        	else if(s=='l'){
				if(angle[1]< 1.5 )
					move(clientID,leftMotorHandle,rightMotorHandle,-1*leftMotorSpeed,rightMotorSpeed);
				else{
					//c = 1;
					move(clientID,leftMotorHandle,rightMotorHandle,0,0);
				}
			}
        	else if(s=='r'){
        		if(angle[1]> -1.5 )
					move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,-1*rightMotorSpeed);
				else{
					//c =1;
					move(clientID,leftMotorHandle,rightMotorHandle,0,0);
				}
        	}
        	else{ 
        		printf("invalid input\n");
        		break;
        	}
        	/*if(c==1)
        	{
        		simxSetObjectOrientation(clientID,reference,botbase,angle,simx_opmode_oneshot);
				printf("reference orientation: ");
				printf("the angle: %f %f %f\n",angle[0],angle[1],angle[2]);
				c =0;
        	}*/
        	extApi_sleepMs(25);
        	        /*********************************************************/
        }
        
        // end communication with lua
        simxFinish(clientID);

    }
    printf("Simulation Stopped!");
}
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float leftMotorSpeed,float rightMotorSpeed)
{
	simxSetJointTargetVelocity(clientID,leftMotorHandle,leftMotorSpeed,simx_opmode_oneshot);
	simxSetJointTargetVelocity(clientID,rightMotorHandle,rightMotorSpeed,simx_opmode_oneshot);
}
