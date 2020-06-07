#include "vrep.h"

// for more info refer to 
// https://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm 

int main(int argc,char* argv[])
{

    // all handles that will be passed from lua to this code
    int portNb=0;
    int botHandle;
    int gspotHandle;
    int leftMotorHandle;
    int rightMotorHandle;  
    int sensorHandle;
    
    // number of variables passed from lua to this code + 1
    // (+1 because the first value in argv is always the location of this program)
    int n = 7;

    // save the handles
    if (argc>=n){
        portNb=atoi(argv[1]);
        botHandle = atoi(argv[2]);
        gspotHandle = atoi(argv[3]);    
        leftMotorHandle = atoi(argv[4]);
        rightMotorHandle = atoi(argv[5]);
        sensorHandle = atoi(argv[6]);
    }
    else{
        printf("Sufficient arguments not provided'!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // variables for the code
    float position[3] = {0,0,0};
    float leftMotorSpeed = 10000000;
    float rightMotorSpeed = 1;
    float* auxValues = NULL;
    int* auxValuesCount = NULL;

    // functions or variable names starting with 'simx' are coppeliamsim functions
    // for details on these functions refer: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm

    // start communication with coppeliasim
    //int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);

    if (clientID!=-1){
        printf("connection established\n");

        /******************** SETUP ******************************/
        // write code here that you wish to perform ONCE
        simxGetObjectPosition(clientID, gspotHandle, -1, position, simx_opmode_streaming);
        simxSetJointTargetVelocity(clientID,leftMotorHandle,leftMotorSpeed,simx_opmode_oneshot);
        simxSetJointTargetVelocity(clientID,rightMotorHandle,rightMotorSpeed,simx_opmode_oneshot);
        simxReadVisionSensor(clientID, sensorHandle, NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
        /*********************************************************/
        
        while (simxGetConnectionId(clientID)!=-1){  
            /************************ LOOP *****************************/
            // write code here that you wish to REPEAT
            if( simxGetObjectPosition(clientID, gspotHandle, -1, position, simx_opmode_buffer) == simx_return_ok)
                //printf("the pos: %f %f %f\n",position[0],position[1],position[2]);
            if( simxReadVisionSensor(clientID, sensorHandle, NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok){
                //printf("sensor: %f\n", auxValues[10]);
            }
            /***********************************************************/
        }
        
        // end communication with lua
        simxFinish(clientID);

    }
    printf("Simulation Stopped!");
}
