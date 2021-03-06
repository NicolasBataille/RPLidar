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
        int cpt = 0; //on cr???? un compteur pour voir combien d'??l??ments sont donn??s par le Lidar

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

            int quart = cpt/4;  //permet d'avoir la sectorisation en quartiers de l'analyde du Lidar
            int treshold_distance = 300;    //valeurs seuil de distance pour les test (unit?? ?? d??finir)

            bool front, right, back, left = false;  //set les bool??ennes utilis??es pour les messages de test

            //Front part
            for(int i = 0; i < quart; i++){
                if((nodes[i].distance_q2/4.0f <= treshold_distance) && (nodes[i].distance_q2/4.0f != 0)){
                    front = true;
                    std::cout << nodes[i].distance_q2/4.0f << std::endl;
                    break;
                }
            }

            //Right part
            for(int i = quart; i < quart*2; i++){
                if((nodes[i].distance_q2/4.0f <= treshold_distance) && (nodes[i].distance_q2/4.0f != 0)){
                    right = true;
                    std::cout << nodes[i].distance_q2/4.0f << std::endl;
                    break;
                }
            }

            //Back part
            for(int i = quart*2; i < quart*3; i++){
                if((nodes[i].distance_q2/4.0f <= treshold_distance) && (nodes[i].distance_q2/4.0f != 0)){
                    back = true;
                    std::cout << nodes[i].distance_q2/4.0f << std::endl;
                    break;
                }
            }

            //Left part
            for(int i = quart*3; i < quart*4; i++){
                if((nodes[i].distance_q2/4.0f <= treshold_distance) && (nodes[i].distance_q2/4.0f != 0)){
                    left = true;
                    std::cout << nodes[i].distance_q2/4.0f << std::endl;
                    break;
                }
            }

            if(front){
                std::cout << "Un obstacle est devant" << std::endl;
                front = false;
            }
            else{
                std::cout << "Rien devant" << std::endl;
            }

            if(right){
                std::cout << "Un obstacle est ?? droite" << std::endl;
                right = false;
            }
            else{
                std::cout << "Rien ?? droite" << std::endl;
            }

            if(back){
                std::cout << "Un obstacle est derri??re" << std::endl;
                back = false;
            }
            else{
                std::cout << "Rien derri??re" << std::endl;
            }

            if(left){
                std::cout << "Un obstacle est ?? gauche" << std::endl;
                left = false;
            }
            else{
                std::cout << "Rien ?? gauche" << std::endl;
            }

/*            front, right, back, left = false;   //reset des bool??ennes
*/
            cpt = 0;    //remet le compteur ?? 0 pour la prochaine analyse

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