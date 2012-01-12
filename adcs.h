#ifndef ADCS_H
#define ADCS_H




// SIMULATION STATE DEFINES
#define STOPPED 0
#define RUNNING 1


// CONTROLLER MODE DEFINES
#define FINE    0
#define COARSE  1




class adcs
{



public:

    // Constructor function
    adcs();



    /************* SET FUNCTIONS *************/

    void setRoll(double roll);

    void setPitch(double pitch);

    void setYaw(double yaw);

    void setSimulationState(short state);

    void setControllerMode(short mode);

    void setCurrentQuaternions(double q0, double q1, double q2, double q3);

    void setWheelSpeeds(int w1, int w2, int w3);

    void setMagnetometer(double arg1, double arg2, double arg3);

    void setGyroscope(double arg1, double arg2, double arg3);

    void setAccelerometer(double arg1, double arg2, double arg3);

    void setIR_blob(int num, int coord, int value);




    /************* GET FUNCTIONS *************/

    double getRoll();

    double getPitch();

    double getYaw();

    double getQuaternion(int n);

    double getCurrentQuaternion(int n);

    int getWheelSpeed(int n);

    double getMagnetometer(int n);

    double getGyroscope(int n);

    double getAccelerometer(int n);

    short getSimulationState();

    short getControllerMode();

    int getIR_blob(int num, int coord);



private:

    void computeQuaternions();
    
    

    short simulationState;

    short controllerMode;


    double roll, pitch, yaw;

    double quaternions[4];

    double currentQuaternions[4];


    int wheelSpeeds[3];

    double magnetometer[3];

    double gyroscope[3];

    double accelerometer[3];

    int IR_blob[4][2];

};

#endif // ADCS_H
