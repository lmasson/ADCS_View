#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "adcs.h"
#include "widget/core/ConnectDialog.h"

#include "communication_protocol.h"
#include "flags.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>


//#define WIN_32        // Uncomment this line if compiling on a Windows architecture


// Define libraries for communication protocol (sockets, etc.)
#ifdef WIN_32
   //#include <winsock.h>
   #include <winsock2.h>
#else
   #include <netinet/in.h>
   #include <sys/socket.h>
   #include <arpa/inet.h>
#endif


// Global variables
adcs* platform;
QTimer* timer;
QPen pen = QPen();
QBrush brush = QBrush();




//******************************************
// TEMPORARY

pthread_t thread_c;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void* streamClient(void* arg);
void  quit();

int       message_flag = NO_MESSAGE;
int       is_data_ready = 0;
int       sock;
char*     server_ip;// = "192.168.1.102";
int       server_port;// = 8888;





int twoComplement(uint8_t low, int8_t high);


int connection_state = 0;

//int num = 0;


// tcp:10.0.0.133;port=8888


//******************************************


// Constructor for MainWindow class

MainWindow::MainWindow(QWidget *parent) :    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Connections
    connect(ui->connectAction, SIGNAL(triggered()), this, SLOT(openConnectDialog()));

    // Construct an 'adcs' class object
    platform = new adcs;

    // Initialize QGraphicScene representing the position of the IR blobs
    IR_scene = new QGraphicsScene;

    // Initialize red pen
    pen.setBrush(Qt::red);
    pen.setWidth(1);

    // Initialize the 4 rectangles at each corner of the QGraphicScene
    IR_dot1 = IR_scene->addRect( (qreal)10, (qreal)10, (qreal)2, (qreal)2, pen, brush );
    IR_dot2 = IR_scene->addRect( (qreal)113, (qreal)10, (qreal)2, (qreal)2, pen, brush );
    IR_dot3 = IR_scene->addRect( (qreal)10, (qreal)113, (qreal)2, (qreal)2, pen, brush );
    IR_dot4 = IR_scene->addRect( (qreal)113, (qreal)113, (qreal)2, (qreal)2, pen, brush );
    ui->IR_graphics->setScene(IR_scene);

    // Initialise display of quaternion values
    this->updateQuaternions();

    // String containing the server IP
    server_ip = (char*)malloc(20*sizeof(char));

    // Initialize timer for the refresh of the ADCS platform status
    timer = new QTimer(this);
    timer->setSingleShot(false);
    timer->start(50);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCaption()));
}



// Destructor for MainWindow class

MainWindow::~MainWindow()
{
    delete ui;
    quit();
}





void MainWindow::updateCaption()
{
    // Update display of current quaternions
    ui->cur_q0_spinbox->setValue(platform->getCurrentQuaternion(0));
    ui->cur_q1_spinbox->setValue(platform->getCurrentQuaternion(1));
    ui->cur_q2_spinbox->setValue(platform->getCurrentQuaternion(2));
    ui->cur_q3_spinbox->setValue(platform->getCurrentQuaternion(3));

    // Update display of connexion status
    if(connection_state == 0)
        ui->state_label2->setText("<font color='red'>Disconnected</font>");
    else
        ui->state_label2->setText("<font color='green'>Connected</font>");

    // Update display of accelerometers, gyroscopes, and magnetometers
    updateWheelSpeedDisplay();
    updateAccelerometerDisplay();
    updateGyroscopeDisplay();
    updateMagnetometerDisplay();

    // Display peaks detected by the IR sensor (if X = 1023 and Y = 1023, they
    // are not detected, and thus won't be displayed.
    if(connection_state == 1)
    {
        if( platform->getIR_blob(0,0) >= 1023 && platform->getIR_blob(0,1) >= 1023 )
            IR_dot1->setRect( (qreal) (platform->getIR_blob(0,0)), (qreal) (platform->getIR_blob(0,1)), (qreal)2, (qreal)2 );
        else
            IR_dot1->setRect( (qreal) (platform->getIR_blob(0,0))/10, (qreal) (platform->getIR_blob(0,1))/10, (qreal)2, (qreal)2 );

        if( platform->getIR_blob(1,0) >= 1023 && platform->getIR_blob(1,1) >= 1023 )
            IR_dot2->setRect( (qreal) (platform->getIR_blob(1,0)), (qreal) (platform->getIR_blob(1,1)), (qreal)2, (qreal)2 );
        else
            IR_dot2->setRect( (qreal) (platform->getIR_blob(1,0))/10, (qreal) (platform->getIR_blob(1,1))/10, (qreal)2, (qreal)2 );

        if( platform->getIR_blob(2,0) >= 1023 && platform->getIR_blob(2,1) >= 1023 )
            IR_dot3->setRect( (qreal) (platform->getIR_blob(2,0)), (qreal) (platform->getIR_blob(2,1)), (qreal)2, (qreal)2 );
        else
            IR_dot3->setRect( (qreal) (platform->getIR_blob(2,0))/10, (qreal) (platform->getIR_blob(2,1))/10, (qreal)2, (qreal)2 );

        if( platform->getIR_blob(3,0) >= 1023 && platform->getIR_blob(3,1) >= 1023 )
            IR_dot4->setRect( (qreal) (platform->getIR_blob(3,0)), (qreal) (platform->getIR_blob(3,1)), (qreal)2, (qreal)2 );
        else
            IR_dot4->setRect( (qreal) (platform->getIR_blob(3,0))/10, (qreal) (platform->getIR_blob(3,1))/10, (qreal)2, (qreal)2 );
    }
}



// Updates the display of the Wheel Speed values

void MainWindow::updateWheelSpeedDisplay()
{
    int i;

    i = platform->getWheelSpeed(0);
    if(i >= 0)
    {
        ui->progressBarAccXNeg->setValue(0);
        ui->progressBarAccXPos->setValue(i);
    }
    else
    {
        ui->progressBarAccXPos->setValue(0);
        ui->progressBarAccXNeg->setValue(abs(i));
    }

    i = platform->getWheelSpeed(1);
    if( i >= 0 )
    {
        ui->progressBarAccYNeg->setValue(0);
        ui->progressBarAccYPos->setValue(i);
    }
    else
    {
        ui->progressBarAccYPos->setValue(0);
        ui->progressBarAccYNeg->setValue(abs(i));
    }

    i = platform->getWheelSpeed(2);
    if( i >= 0 )
    {
        ui->progressBarAccZNeg->setValue(0);
        ui->progressBarAccZPos->setValue(i);
    }
    else
    {
        ui->progressBarAccZPos->setValue(0);
        ui->progressBarAccZNeg->setValue(abs(i));
    }
}



// Updates the display of the Accelerometer values

void MainWindow::updateAccelerometerDisplay()
{
    int i;

    ui->accelX_spinbox->setValue(platform->getAccelerometer(0));
    ui->accelY_spinbox->setValue(platform->getAccelerometer(1));
    ui->accelZ_spinbox->setValue(platform->getAccelerometer(2));


    i = 1000*platform->getAccelerometer(0);
    if(i >= 0)
    {
        ui->progressBarAccXNeg->setValue(0);
        ui->progressBarAccXPos->setValue(i);
    }
    else
    {
        ui->progressBarAccXPos->setValue(0);
        ui->progressBarAccXNeg->setValue(abs(i));
    }

    i = 1000*platform->getAccelerometer(1);
    if( i >= 0 )
    {
        ui->progressBarAccYNeg->setValue(0);
        ui->progressBarAccYPos->setValue(i);
    }
    else
    {
        ui->progressBarAccYPos->setValue(0);
        ui->progressBarAccYNeg->setValue(abs(i));
    }

    i = 1000*platform->getAccelerometer(2);
    if( i >= 0 )
    {
        ui->progressBarAccZNeg->setValue(0);
        ui->progressBarAccZPos->setValue(i);
    }
    else
    {
        ui->progressBarAccZPos->setValue(0);
        ui->progressBarAccZNeg->setValue(abs(i));
    }
}


// Updates the display of the Gyroscope values

void MainWindow::updateGyroscopeDisplay()
{
    int i;

    ui->gyroX_spinbox->setValue(platform->getGyroscope(0));
    ui->gyroY_spinbox->setValue(platform->getGyroscope(1));
    ui->gyroZ_spinbox->setValue(platform->getGyroscope(2));

    i = platform->getGyroscope(0);
    if(i >= 0)
    {
        ui->progressBarGyroXNeg->setValue(0);
        ui->progressBarGyroXPos->setValue(i);
    }
    else
    {
        ui->progressBarGyroXPos->setValue(0);
        ui->progressBarGyroXNeg->setValue(abs(i));
    }

    i = platform->getGyroscope(1);
    if( i >= 0 )
    {
        ui->progressBarGyroYNeg->setValue(0);
        ui->progressBarGyroYPos->setValue(i);
    }
    else
    {
        ui->progressBarGyroYPos->setValue(0);
        ui->progressBarGyroYNeg->setValue(abs(i));
    }

    i = platform->getGyroscope(2);
    if( i >= 0 )
    {
        ui->progressBarGyroZNeg->setValue(0);
        ui->progressBarGyroZPos->setValue(i);
    }
    else
    {
        ui->progressBarGyroZPos->setValue(0);
        ui->progressBarGyroZNeg->setValue(abs(i));
    }
}


// Updates the display of the magnetometer values

void MainWindow::updateMagnetometerDisplay()
{
    int i;

    ui->magX_spinbox->setValue(platform->getMagnetometer(0));
    ui->magY_spinbox->setValue(platform->getMagnetometer(1));
    ui->magZ_spinbox->setValue(platform->getMagnetometer(2));

    i = platform->getMagnetometer(0);
    if(i >= 0)
    {
        ui->progressBarMagXNeg->setValue(0);
        ui->progressBarMagXPos->setValue(i);
    }
    else
    {
        ui->progressBarMagXPos->setValue(0);
        ui->progressBarMagXNeg->setValue(abs(i));
    }

    i = platform->getMagnetometer(1);
    if( i >= 0 )
    {
        ui->progressBarMagYNeg->setValue(0);
        ui->progressBarMagYPos->setValue(i);
    }
    else
    {
        ui->progressBarMagYPos->setValue(0);
        ui->progressBarMagYNeg->setValue(abs(i));
    }

    i = platform->getMagnetometer(2);
    if( i >= 0 )
    {
        ui->progressBarMagZNeg->setValue(0);
        ui->progressBarMagZPos->setValue(i);
    }
    else
    {
        ui->progressBarMagZPos->setValue(0);
        ui->progressBarMagZNeg->setValue(abs(i));
    }
}






// Opens Connect Dialog window after having clicked on the 'Connect' menu option

void MainWindow::openConnectDialog()
{
    ConnectDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted){
        ui->Connect2->setText(dialog.connectionTarget());
        int arg1, arg2, arg3, arg4;
        sscanf(dialog.connectionTarget().toStdString().c_str(),"tcp:%d.%d.%d.%d;port=%d",&arg1, &arg2, &arg3, &arg4, &server_port);
        sprintf(server_ip,"%d.%d.%d.%d",arg1, arg2, arg3, arg4);

        // Run the streaming client as a separate thread
        if (pthread_create(&thread_c, NULL, streamClient, NULL)) {
         //   quit();
        }
    }
}







void* streamClient(void* arg)
{
    struct  sockaddr_in server;

    char sendBuffer[30];
    char buffer[42];

    double  i, j, k, l;
    int bytes, xValue, yValue;

    // Make this thread cancellable using pthread_cancel()
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    // Create a socket for Windows environment
    #ifdef WIN_32
    WSADATA wsadata;
    if (WSAStartup(MAKEWORD(1,1), &wsadata) == SOCKET_ERROR) {
        printf("Error creating socket.");
        return 0;
    }
    #endif

    // Create a socket for Unix environment
    if ((sock = socket(PF_INET,SOCK_STREAM, 0)) < 0) {
    //    quit();
    }

    // Setup server parameters
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(server_ip);
    server.sin_port = htons(server_port);

    // Connect to server on Overo board
    if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
     //   quit();
    }

    // Set connection state flag to 1
    connection_state = 1;


    // Start receiving images
    while(1)
    {
        // Lock thread
        pthread_mutex_lock(&mutex);

        // Receive data from the Overo board
        if ((bytes = recv(sock, buffer, sizeof(buffer) , 0)) == -1) {
          //  quit();
        }

        // Accelerometer readings
        i = (double) twoComplement(buffer[0], buffer[1])/16384;
        j = (double) twoComplement(buffer[2], buffer[3])/16384;
        k = (double) twoComplement(buffer[4], buffer[5])/16384;
        platform->setAccelerometer(i,j,k);
        
        // Magnetometer readings
        i = (double) twoComplement(buffer[6], buffer[7]);
        j = (double) twoComplement(buffer[8], buffer[9]);
        k = (double) twoComplement(buffer[10] ,buffer[11]);
        platform->setMagnetometer(i,j,k);

        // Gyroscope readings
        i = (double) twoComplement(buffer[12], buffer[13])*500.0/16384;
        j = (double) twoComplement(buffer[14], buffer[15])*500.0/16384;
        k = (double) twoComplement(buffer[16], buffer[17])*500.0/16384;
        platform->setGyroscope(i,j,k);

        // Quaternion readings
        i = (double) twoComplement(buffer[18], buffer[19])/10000;
        j = (double) twoComplement(buffer[20], buffer[21])/10000;
        k = (double) twoComplement(buffer[22], buffer[23])/10000;
        l = (double) twoComplement(buffer[24], buffer[25])/10000;
        platform->setCurrentQuaternions(i, j, k, l);

        // IR Camera peaks readings
        xValue = (int) twoComplement(buffer[26], buffer[27]);
        yValue = (int) twoComplement(buffer[28], buffer[29]);
        platform->setIR_blob(0,0,xValue);
        platform->setIR_blob(0,1,yValue);
        xValue = (int) twoComplement(buffer[30], buffer[31]);
        yValue = (int) twoComplement(buffer[32], buffer[33]);
        platform->setIR_blob(1,0,xValue);
        platform->setIR_blob(1,1,yValue);
        xValue = (int) twoComplement(buffer[34], buffer[35]);
        yValue = (int) twoComplement(buffer[36], buffer[37]);
        platform->setIR_blob(2,0,xValue);
        platform->setIR_blob(2,1,yValue);
        xValue = (int) twoComplement(buffer[38], buffer[39]);
        yValue = (int) twoComplement(buffer[40], buffer[41]);
        platform->setIR_blob(3,0,xValue);
        platform->setIR_blob(3,1,yValue);

        // Wheel speed readings
        i = (double) twoComplement(buffer[42], buffer[43]);
        j = (double) twoComplement(buffer[44], buffer[45]);
        k = (double) twoComplement(buffer[46], buffer[47]);
        platform->setWheelSpeeds(i, j, k);


        // Prepare the sendBuffer accordingly to the message_flag
        switch(message_flag)
        {

        case NO_MESSAGE:
            emptyBuffer(sendBuffer);
            break;

        case SET_SIMULATION_STATE:
            setMessage_simulationState(sendBuffer, platform->getSimulationState());
            break;

        case SET_QUATERNIONS:
            setMessage_setQuaternions(sendBuffer, platform->getQuaternion(0), platform->getQuaternion(1), platform->getQuaternion(2), platform->getQuaternion(3));
            break;

        case SET_ALIGNMENT_PRECISION:
            setMessage_setAlignmentPrecision(sendBuffer, platform->getControllerMode());
            break;

        case SET_MOTOR_DEBUG:
            setMessage_motorDebug(sendBuffer, 1);
            break;

        }

        // Send awaiting message
        bytes = send(sock, sendBuffer, sizeof(sendBuffer), 0);

        // Unlock thread
        pthread_mutex_unlock(&mutex);

        // Return the message flag to the NO_MESSAGE state
        message_flag = NO_MESSAGE;

        // Clear the message buffer
        emptyBuffer(sendBuffer);
    }
}



int twoComplement(uint8_t low, int8_t high)
{
    return low+(high << 8);
}



// This function provides a way to exit nicely from the system

void quit()
{
    // Close socket
    if (sock) close(sock);

    // Set connection state flag to 0
    connection_state = 0;

    // Close streaming client thread
    pthread_mutex_destroy(&mutex);

    //exit(retval);
    exit(0);
}




// Start / run simulation button

void MainWindow::on_simulationButton_clicked()
{
    if( platform->getSimulationState() == STOPPED )
    {
        ui->simulationButton->setText("Stop Simulation");
        platform->setSimulationState(RUNNING);
    }
    else if( platform->getSimulationState() == RUNNING )
    {
        ui->simulationButton->setText("Start Simulation");
        platform->setSimulationState(STOPPED);
    }

    // Set message flag
    message_flag = SET_SIMULATION_STATE;
}


// Update of roll / pitch / yaw after a change of value in the spinboxes

void MainWindow::on_roll_spinbox_valueChanged(double arg1)
{
    platform->setRoll(arg1);
    ui->roll_dial->setValue((int)100*platform->getRoll());

    // Set message flag
    message_flag = SET_QUATERNIONS;
}

void MainWindow::on_pitch_spinbox_valueChanged(double arg1)
{
    platform->setPitch(arg1);
    ui->pitch_dial->setValue((int)100*platform->getPitch());

    // Set message flag
    message_flag = SET_QUATERNIONS;
}

void MainWindow::on_yaw_spinbox_valueChanged(double arg1)
{
    platform->setYaw(arg1);
    ui->yaw_dial->setValue((int)100*platform->getYaw());

    // Set message flag
    message_flag = SET_QUATERNIONS;
}



// Update of roll / pitch / yaw after a change of value in the dials

void MainWindow::on_roll_dial_valueChanged(int value)
{
    platform->setRoll((float)value/100);
    ui->roll_spinbox->setValue(platform->getRoll());
    this->updateQuaternions();

    // Set message flag
    message_flag = SET_QUATERNIONS;
}

void MainWindow::on_pitch_dial_valueChanged(int value)
{
    platform->setPitch((float)value/100);
    ui->pitch_spinbox->setValue(platform->getPitch());
    this->updateQuaternions();

    // Set message flag
    message_flag = SET_QUATERNIONS;
}

void MainWindow::on_yaw_dial_valueChanged(int value)
{
    platform->setYaw((float)value/100);
    ui->yaw_spinbox->setValue(platform->getYaw());
    this->updateQuaternions();

    // Set message flag
    message_flag = SET_QUATERNIONS;
}







// Updates the display of the command quaternions after having modified one
// of the Roll / Pitch / Yaw dials or spinboxes

void MainWindow::updateQuaternions()
{
    ui->q0_spinbox->setValue(platform->getQuaternion(0));
    ui->q1_spinbox->setValue(platform->getQuaternion(1));
    ui->q2_spinbox->setValue(platform->getQuaternion(2));
    ui->q3_spinbox->setValue(platform->getQuaternion(3));
}






// Switches the Alignment precision mode from FINE to COARSE

void MainWindow::on_coarseAlignment_radiobutton_released()
{
    platform->setControllerMode(COARSE);
    ui->fineAlignment_radiobutton->setChecked(false);

    // Set message flag
    message_flag = SET_ALIGNMENT_PRECISION;
}


// Switches the Alignment precision mode from COARSE to FINE

void MainWindow::on_fineAlignment_radiobutton_released()
{
    platform->setControllerMode(FINE);
    ui->coarseAlignment_radiobutton->setChecked(false);

    // Set message flag
    message_flag = SET_ALIGNMENT_PRECISION;
}





