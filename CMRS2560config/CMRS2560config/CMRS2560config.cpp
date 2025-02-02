// CMRS2560config.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include "CMRSconfig.h"

CMRSconfig TheConfig;

std::string getSignalAspect(int arg)
{
    std::string ret_val;
    switch (arg) {
    case 1:
        ret_val = "Dark";
        break;
    case 2:
        ret_val = "Red";
        break;
    case 3:
        ret_val = "Flashing Red";
        break;
    case 4:
        ret_val = "Lunar";
        break;
    case 5:
        ret_val = "Flashing Lunar";
        break;
    case 6:
        ret_val = "Yellow";
        break;
    case 7:
        ret_val = "Flashing Yellow";
        break;
    case 8:
        ret_val = "Green";
        break;
    case 9:
        ret_val = "Flashing Green";
        break;
    default:
        ret_val = "UNKNOWN";
        break;
    }
    return ret_val;
}

int getData()
{
//   FILE* fp = fopen("..\\..\\..\\..\\..\\CMRS_CP_2560\\station-conf\\64.TXT", "r");
    
    std::ifstream file1("..\\..\\..\\CMRS_CP_2560\\station-conf\\64.TXT");

    if (!(file1.is_open())) {
        std::perror("Can't open file");
        return 1;
    }

    std::string dataIn;

    while (std::getline(file1, dataIn)) {
 //       std::cout << dataIn << std::endl;
        TheConfig.parse_data(dataIn);
    }
 //   fclose(fp);
    file1.close();

    return 0;
}



int main(int argc, char* argv[])
{
    std::cout << "Number of arguments " << argc << std::endl;
    
 //   TheConfig = new CMRSconfig();

    getData();

    TheConfig.printMacAddr();
    TheConfig.printIpAddr();
    TheConfig.printServerIpAddr();
    TheConfig.printBoards();
    TheConfig.printQuadTurnouts();
    TheConfig.printQuadSensors();
    TheConfig.printDualTurnouts();
    TheConfig.printSensors();
    TheConfig.printSignals();
    TheConfig.printSignalInputs();
    TheConfig.printSignalLogic();
    TheConfig.printIndicators();
    TheConfig.printKBTrack();

    std::cout << "Continue...";

    TheConfig.writeData();
    
 //   std::string cmdIn;
 //   std::cin >> cmdIn;

}
