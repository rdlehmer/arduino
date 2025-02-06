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

void printConfiguration() {
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
}


int writeDataFile() {
    TheConfig.writeData();
    return(0);
}

void showDataDispatcher() {
    std::string arg;
    std::cin >> arg;

    if (arg.compare(0, 2, "conf", 0, 2) == 0) {
        printConfiguration();
    }
    else if (arg.compare(0, 2, "mac", 0, 2) == 0) {
        TheConfig.printMacAddr();
    }
    else if (arg.compare(0, 2, "ip", 0, 2) == 0) {
        TheConfig.printIpAddr();
    }
    else if (arg.compare(0, 3, "server", 0, 3) == 0) {
        TheConfig.printServerIpAddr();
    }
    else if (arg.compare(0, 2, "board", 0, 2) == 0) {
        TheConfig.printBoards();
    }
    else if (arg.compare(0, 2, "turnout", 0, 2) == 0) {
        TheConfig.printQuadTurnouts();
    }
    else if (arg.compare(0, 2, "qsensors", 0, 2) == 0) {
        TheConfig.printQuadSensors();
    }
    else if (arg.compare(0, 2, "dual", 0, 2) == 0) {
        TheConfig.printDualTurnouts();
    }
    else if (arg.compare(0, 3, "sensors", 0, 3) == 0) {
        TheConfig.printSensors();
    }
    else if (arg.compare(0, 2, "signals", 0, 2) == 0) {
        TheConfig.printSignals();
    }
    else if (arg.compare(0, 3, "inputs", 0, 3) == 0) {
        TheConfig.printSignalInputs();
    }
    else if (arg.compare(0, 2, "logic", 0, 2) == 0) {
        TheConfig.printSignalLogic();
    }
    else if (arg.compare(0, 3, "indicators", 0, 3) == 0) {
        TheConfig.printIndicators();
    }
    else if (arg.compare(0, 2, "keyboard", 0, 2) == 0) {
        TheConfig.printKBTrack();
    }
    else {
        std::cout << "%%show - syntax error." << std::endl;
    }
}

void setMacAddr() {
    int macaddr[6];
    std::string arg;
    std::cin >> arg;
    if (arg.length() == 17) {
        // format of data xx-xx-xx-xx-xx-xx-xx
        for (int i = 0; i < 6; i++) {
            std::string sval = arg.substr(3 * i, 2);
            macaddr[i] = std::stoi(sval, nullptr, 16);
        }
        TheConfig.setMacAddr(macaddr);
    }
    else {
        std::cout << "%%setMacAddr - syntax error" << std::endl;
    }
}

void setIpAddr() {
    int ipaddr[4];
    std::string arg;
    std::cin >> arg;
    int startpos = 0;
    for (int i = 0; i < 3; i++) {
        int index = arg.find_first_of(".", startpos);
        std::string sval = arg.substr(startpos, index - startpos);
        ipaddr[i] = std::stoi(sval, nullptr, 10);
        startpos = index+1;
    }
    std::string sval = arg.substr(startpos, arg.length() - startpos);
    ipaddr[3] = std::stoi(sval, nullptr, 10);
    TheConfig.setIpAddr(ipaddr);
}

void setServerIpAddr() {
    int ipaddr[4];
    std::string arg;
    std::cin >> arg;
    int startpos = 0;
    for (int i = 0; i < 3; i++) {
        int index = arg.find_first_of(".", startpos);
        std::string sval = arg.substr(startpos, index - startpos);
        ipaddr[i] = std::stoi(sval, nullptr, 10);
        startpos = index + 1;
    }
    std::string sval = arg.substr(startpos, arg.length() - startpos);
    ipaddr[3] = std::stoi(sval, nullptr, 10);
    TheConfig.setServerIpAddr(ipaddr);
}

void setBoard() {
    int index = -1;
    int val = 0;
    std::string arg;
    std::cin >> arg;
    if (arg.compare(0, 2, "toggle", 0, 2) == 0) {
        index = 0;
    }
    else if (arg.compare(0, 2, "sensor", 0, 2) == 0) {
        index = 1;
    }
    else if (arg.compare(0, 2, "turnout", 0, 2) == 0) {
        index = 2;
    }
    else if (arg.compare(0, 2, "indicator", 0, 2) == 0) {
        index = 3;
    }
    else if (arg.compare(0, 2, "signal", 0, 2) == 0) {
        index = 4;
    }
    else if (arg.compare(0, 2, "dual", 0, 2) == 0) {
        index = 5;
    }
    else if (arg.compare(0, 2, "relay", 0, 2) == 0) {
        index = 6;
    }
    else {
        std::cout << "%%setBoard - syntax error" << std::endl;
        return;
    }

    std::cin >> arg;
    val = std::stoi(arg, nullptr, 10);
    TheConfig.setBoard(index, val);
}

void setTurnout() {

}

void setDataDispatcher() {
    std::string arg;
    std::cin >> arg;

    if (arg.compare(0, 2, "mac", 0, 2) == 0) {
        setMacAddr();
    }
    else if (arg.compare(0, 2, "ip", 0, 2) == 0) {
        setIpAddr();
    }
    else if (arg.compare(0, 3, "server", 0, 3) == 0) {
        setServerIpAddr();
    }
    else if (arg.compare(0, 2, "board", 0, 2) == 0) {
        setBoard();
    }
    else if (arg.compare(0, 2, "turnout", 0, 2) == 0) {
        setTurnout();
//        TheConfig.printQuadTurnouts();
    }
    else if (arg.compare(0, 2, "qsensors", 0, 2) == 0) {
//        TheConfig.printQuadSensors();
    }
    else if (arg.compare(0, 2, "dual", 0, 2) == 0) {
//        TheConfig.printDualTurnouts();
    }
    else if (arg.compare(0, 3, "sensors", 0, 3) == 0) {
//        TheConfig.printSensors();
    }
    else if (arg.compare(0, 2, "signals", 0, 2) == 0) {
//        TheConfig.printSignals();
    }
    else if (arg.compare(0, 3, "inputs", 0, 3) == 0) {
//        TheConfig.printSignalInputs();
    }
    else if (arg.compare(0, 2, "logic", 0, 2) == 0) {
//        TheConfig.printSignalLogic();
    }
    else if (arg.compare(0, 3, "indicators", 0, 3) == 0) {
//        TheConfig.printIndicators();
    }
    else if (arg.compare(0, 2, "keyboard", 0, 2) == 0) {
//        TheConfig.printKBTrack();
    }
    else {
        std::cout << "%%set - syntax error." << std::endl;
    }
}

int commandLine() {
    static int requestRun = 1;
    std::string cmdIn;
    char temp[256];
    while (requestRun == 1) {
        std::cout << "Config> ";
        std::cin >> cmdIn;

        std::string cmdRoot = cmdIn.substr(0, cmdIn.find(" "));

        if (cmdIn.compare("exit") == 0) {
            requestRun = 0;
        }
        else if (cmdIn.compare("open") == 0) {
            std::string fileName;
            std::cin >> fileName;
            std::cout << fileName << std::endl;
        }
        else if (cmdIn.compare("show") == 0) {
            showDataDispatcher();
        }
        else if (cmdIn.compare("set") == 0) {
            setDataDispatcher();
        }
    }

    return(0);
}

int main(int argc, char* argv[])
{
    std::cout << "Number of arguments " << argc << std::endl;
    
 //   TheConfig = new CMRSconfig();

    getData();

    commandLine();

    std::cout << "Exiting...";



 //   std::string cmdIn;
 //   std::cin >> cmdIn;

}
