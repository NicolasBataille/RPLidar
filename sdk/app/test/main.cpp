#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int treshold_distance = 300; 

void check_obstacle(std::string s, int treshold, int cpt, rplidar_response_measurement_node_t* nodes){
                std::string present, notpresent="";
                int lowerbound, upperbound = 0;
                int section = cpt/8;
                int decalangle = section/2;
                bool spotted=false;
                bool north=false;

                if (s=="N"){
                    present="Un obstacle est a l'avant";
                    notpresent="Il n'y a rien devant";
                    lowerbound=cpt-decalangle;
                    upperbound=section-decalangle;
                    north=true;

                }
                if (s=="E"){
                    present="Un obstacle est a droite";
                    notpresent="Il n'y a rien a droite";
                    lowerbound=section*2-decalangle;
                    upperbound=section*3-decalangle;
                }
                if (s=="S"){
                    present="Un obstacle est derrière";
                    notpresent="Il n'y a rien derrière";
                    lowerbound=section*4-decalangle;
                    upperbound=section*5-decalangle;
                }
                if (s=="O"){
                    present="Un obstacle est a gauche";
                    notpresent="Il n'y a rien a gauche";
                    lowerbound=section*6-decalangle;
                    upperbound=section*7-decalangle;
                }
                if (s=="NE"){
                    present="Un obstacle est a l'avant droite";
                    notpresent="Il n'y a rien a l'avant droite";
                    lowerbound=section-decalangle;
                    upperbound=section*2-decalangle;
                }
                if (s=="SE"){
                    present="Un obstacle est a l'arrière droite";
                    notpresent="Il n'y a rien a l'arrière droite";
                    lowerbound=section*3-decalangle;
                    upperbound=section*4-decalangle;
                }
                if (s=="SO"){
                    present="Un obstacle est a l'arrière gauche";
                    notpresent="Il n'y a rien a l'arrière gauche";
                    lowerbound=section*5-decalangle;
                    upperbound=section*6-decalangle;
                }
                if (s=="NO"){
                    present="Un obstacle est a l'avant gauche";
                    notpresent="Il n'y a rien l'avant gauche";
                    lowerbound=section*7-decalangle;
                    upperbound=section*8-decalangle;
                }

                if (north){
                    north=false;
                    for(int i = lowerbound; i < cpt-1; i++){
                        if((nodes[i].distance_q2/4.0f <= treshold_distance) && (nodes[i].distance_q2/4.0f != 0)){
                            spotted = true;
                            std::cout << nodes[i].distance_q2/4.0f << std::endl;
                            break;
                        }
                    }
                    for(int i = 0; i < upperbound; i++){
                        if((nodes[i].distance_q2/4.0f <= treshold_distance) && (nodes[i].distance_q2/4.0f != 0)){
                            spotted = true;
                            std::cout << nodes[i].distance_q2/4.0f << std::endl;
                            break;
                        }
                    }
                }
                else {
                    for(int i = lowerbound; i < upperbound; i++){
                        if((nodes[i].distance_q2/4.0f <= treshold_distance) && (nodes[i].distance_q2/4.0f != 0)){
                            spotted = true;
                            std::cout << nodes[i].distance_q2/4.0f << std::endl;
                            break;
                        }
                    }
                }

                if(spotted){
                    spotted = false;
                    std::cout << present << std::endl;
                }
                else{
                    std::cout << " " << std::endl;
                }
                
            }


int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    bool useArgcBaudrate = false;

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n");

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }

    if (!opt_com_path) {
    #ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
    #elif __APPLE__
        opt_com_path = "/dev/tty.SLAB_USBtoUART";
    #else
        opt_com_path = "/dev/ttyUSB0";
    #endif
    }

    // create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    
    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if(useArgcBaudrate)
    {
        // If the driver was not instanciated, then we instanciate it here
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

        // If the connection to the given driver works, then we fetch the device's infos    
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            //We set the connection as successful
            if (IS_OK(op_result)) 
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) 
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {
        
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
    drv->startMotor();
    // start scan...
    drv->startScan(0,1);

    // fetech result and print it out...
    while (1) {
        rplidar_response_measurement_node_t nodes[8192];
        size_t   count = _countof(nodes);

        // std::cout << nodes[90].distance_q2/4.0f << std::endl;
        int cpt = 0; //on créé un compteur pour voir combien d'éléments sont donnés par le Lidar

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                // printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    // (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                    // (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                    // nodes[pos].distance_q2/4.0f,
                    // nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

                    cpt++;
            }

            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  Testing part    ////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            treshold_distance = 300;    //valeurs seuil de distance pour les test (en mm selon les test)

            check_obstacle("N", treshold_distance, cpt, nodes);
            check_obstacle("E", treshold_distance, cpt, nodes);
            check_obstacle("S", treshold_distance, cpt, nodes);
            check_obstacle("O", treshold_distance, cpt, nodes);
            check_obstacle("NE", treshold_distance, cpt, nodes);
            check_obstacle("SE", treshold_distance, cpt, nodes);
            check_obstacle("SO", treshold_distance, cpt, nodes);
            check_obstacle("NO", treshold_distance, cpt, nodes);


            cpt = 0;    //remet le compteur à 0 pour la prochaine analyse

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}