// CMRS2560config.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <algorithm>
#include <cctype>
#include <Windows.h>

#ifdef _WIN32
#include <direct.h>
#define mkdir _mkdir
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif
#include <errno.h>

#include "CMRSconfig.h"


CMRSconfig TheConfig;

std::string getSignalAspect(int arg)
{
    std::string ret_val;
    switch (arg) {
    case 1:
        ret_val = "DARK";
        break;
    case 2:
        ret_val = "RED";
        break;
    case 3:
        ret_val = "FLASHING RED";
        break;
    case 4:
        ret_val = "LUNAR";
        break;
    case 5:
        ret_val = "FLASHING LUNAR";
        break;
    case 6:
        ret_val = "YELLOW";
        break;
    case 7:
        ret_val = "FLASHING YELLOW";
        break;
    case 8:
        ret_val = "GREEN";
        break;
    case 9:
        ret_val = "FLASHING GREEN";
        break;
    default:
        ret_val = "UNKNOWN";
        break;
    }
    return ret_val;
}

int getAspectFromName(std::string arg_name) {
    int ret_val = 0;
    if (arg_name.compare("DARK") == 0) {
        ret_val = 1;
    }
    else if (arg_name.compare("RED") == 0) {
        ret_val = 2;
    }
    else if (arg_name.compare("FLASHING RED") == 0) {
        ret_val = 3;
    }
    else if (arg_name.compare("LUNAR") == 0) {
        ret_val = 4;
    }
    else if (arg_name.compare("FLASHING LUNAR") == 0) {
        ret_val = 5;
    }
    else if (arg_name.compare("YELLOW") == 0) {
        ret_val = 6;
    }
    else if (arg_name.compare("FLASHING YELLOW") == 0) {
        ret_val = 7;
    }
    else if (arg_name.compare("GREEN") == 0) {
        ret_val = 8;
    }
    else if (arg_name.compare("FLASHING GREEN") == 0) {
        ret_val = 9;
    }
    return(ret_val);
}

int getData(std::string arg_str) {

//   FILE* fp = fopen("..\\..\\..\\..\\..\\CMRS_CP_2560\\station-conf\\64.TXT", "r");
    
//    std::ifstream file1("..\\..\\..\\CMRS_CP_2560\\station-conf\\64.TXT");

    std::ifstream file1(arg_str.c_str());

    if (!(file1.is_open())) {
        std::cout << "Can't open file " << arg_str << std::endl;
//        std::perror("Can't open file ");
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

void openFile() {
    std::string arg;
    std::cin >> arg;

    std::string workingfile = TheConfig.getWorkingDir() + +"\\" + arg;
    getData(workingfile);

}

void writeConfFile() {
    std::string working_dir = TheConfig.getWorkingDir();
    std::string dir_name = TheConfig.getConfigDir();
    std::string config_file = dir_name + "\\CMRSconfig.dat";
    std::ofstream fsout;
    fsout.open(config_file.c_str(), std::fstream::out);
    fsout << "#WD " << working_dir << std::endl;
    fsout.close();
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
    std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

    if (arg.compare(0, 2, "CONF", 0, 2) == 0) {
        printConfiguration();
    }
    else if (arg.compare(0, 2, "MAC", 0, 2) == 0) {
        TheConfig.printMacAddr();
    }
    else if (arg.compare(0, 2, "IP", 0, 2) == 0) {
        TheConfig.printIpAddr();
    }
    else if (arg.compare(0, 3, "SERVER", 0, 3) == 0) {
        TheConfig.printServerIpAddr();
    }
    else if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
        TheConfig.printBoards();
    }
    else if (arg.compare(0, 2, "TURNOUT", 0, 2) == 0) {
        TheConfig.printQuadTurnouts();
    }
    else if (arg.compare(0, 2, "QXENSORS", 0, 2) == 0) {
        TheConfig.printQuadSensors();
    }
    else if (arg.compare(0, 2, "DUAL", 0, 2) == 0) {
        TheConfig.printDualTurnouts();
    }
    else if (arg.compare(0, 3, "SENSORS", 0, 3) == 0) {
        TheConfig.printSensors();
    }
    else if (arg.compare(0, 2, "SIGNALS", 0, 2) == 0) {
        TheConfig.printSignals();
    }
    else if (arg.compare(0, 3, "INPUTS", 0, 3) == 0) {
        TheConfig.printSignalInputs();
    }
    else if (arg.compare(0, 2, "LOGIC", 0, 2) == 0) {
        TheConfig.printSignalLogic();
    }
    else if (arg.compare(0, 3, "INDICATORS", 0, 3) == 0) {
        TheConfig.printIndicators();
    }
    else if (arg.compare(0, 2, "KEYBOARD", 0, 2) == 0) {
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
    size_t startpos = 0;
    for (int i = 0; i < 3; i++) {
        size_t index = arg.find_first_of(".", startpos);
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
    size_t startpos = 0;
    for (int i = 0; i < 3; i++) {
        size_t index = arg.find_first_of(".", startpos);
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

    std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

    if (arg.compare(0, 2, "TOGGLE", 0, 2) == 0) {
        index = 0;
    }
    else if (arg.compare(0, 2, "SENSOR", 0, 2) == 0) {
        index = 1;
    }
    else if (arg.compare(0, 2, "TURNOUT", 0, 2) == 0) {
        index = 2;
    }
    else if (arg.compare(0, 2, "INDICATOR", 0, 2) == 0) {
        index = 3;
    }
    else if (arg.compare(0, 2, "SIGNAL", 0, 2) == 0) {
        index = 4;
    }
    else if (arg.compare(0, 2, "DUAL", 0, 2) == 0) {
        index = 5;
    }
    else if (arg.compare(0, 2, "RELAY", 0, 2) == 0) {
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
    int board = 0;
    int channel = 0;
    int toggle = -1;
    char arg_in[256];
    char* pch;

    std::string arg = " ";
    std::string turnoutname;

    std::cin.getline(arg_in, 256);
    
    pch = strtok(arg_in, " ");
    while ( pch != NULL ) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);
        if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setTurnout - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "CHANNEL", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch); 
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setTurnout - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "NAME", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            turnoutname = std::string(pch);
            std::transform(turnoutname.begin(), turnoutname.end(), turnoutname.begin(), ::toupper);
        }

        if (arg.compare(0, 2, "TOGGLE", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            toggle = std::stoi(arg, nullptr, 10);
            if (toggle == -1) {
                std::cout << "%%setTurnout - syntax error" << std::endl;
                return;
            }
        }
        pch = strtok(NULL, " ");
    }
    if ((board != 0) && (channel != 0) && (toggle != -1) && (turnoutname.length() != 0) &&
         (channel > 0) && (channel < 5) && (board > 0) && (board < 5)) {
        TheConfig.setQuadTurnout(4 * (board-1) + channel-1, toggle, turnoutname);
    }
    else {
        std::cout << "%%setTurnout - syntax error" << std::endl;
        return;
    }
    
}

void setQuadSensor() {
    int board = 0;
    int channel = 0;
    int sensor = 0;
    char arg_in[256];
    char* pch;

    std::string arg = " ";
    std::string sensorname;

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setQuadSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "CHANNEL", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setQuadSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "NAME", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            sensorname = std::string(pch);
            std::transform(sensorname.begin(), sensorname.end(), sensorname.begin(), ::toupper);
        }

        if (arg.compare(0, 2, "SENSOR", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            sensor = std::stoi(arg, nullptr, 10);
            if (sensor == -1) {
                std::cout << "%%setQuadSensor - syntax error" << std::endl;
                return;
            }
        }
        pch = strtok(NULL, " ");
    }
    if ((board != 0) && (channel != 0) && (sensor != 0) && (sensorname.length() != 0) &&
        (channel > 0) && (channel < 5) && (board > 0) && (board < 5)) {
        TheConfig.setQuadSensor(board, channel, sensor, sensorname);
    }
    else {
        std::cout << "%%setQuadSensor - syntax error" << std::endl;
        return;
    }

}

void setSensor() {
    int board = 0;
    int channel = 0;
    char arg_in[256];
    char* pch;

    std::string arg = " ";
    std::string sensorname;

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "CHANNEL", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "NAME", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            sensorname = std::string(pch);
            std::transform(sensorname.begin(), sensorname.end(), sensorname.begin(), ::toupper);
        }

        pch = strtok(NULL, " ");
    }
    if ((board != 0) && (channel != 0) && (sensorname.length() != 0) &&
        (channel > 0) && (channel < 5) && (board > 0) && (board < 2)) {
        TheConfig.setSensor(board, channel, sensorname);
    }
    else {
        std::cout << "%%setSensor - syntax error" << std::endl;
        return;
    }

}

void setSignal() {
    int board = 0;
    int channel = 0;
    char arg_in[256];
    char* pch;

    std::string arg = " ";
    std::string signalname;
    std::string leadingsignalname;

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setSignal - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "CHANNEL", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setSignal - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "NAME", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            signalname = std::string(pch);
            std::transform(signalname.begin(), signalname.end(), signalname.begin(), ::toupper);
        }

        if (arg.compare(0, 2, "LEAD", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            leadingsignalname = std::string(pch);
            std::transform(leadingsignalname.begin(), leadingsignalname.end(), leadingsignalname.begin(), ::toupper);
        }

        pch = strtok(NULL, " ");
    }
    if ((board != 0) && (channel != 0) && (signalname.length() != 0) &&
        (channel > 0) && (channel < 5) && (board > 0) && (board < 3)) {
        TheConfig.setSignal(board, channel, signalname, leadingsignalname);
    }
    else {
        std::cout << "%%setSignal - syntax error" << std::endl;
        return;
    }

}

void setSignalInput() {
    int _number = 0;
    int mode = 0;
    int index = 0;
    char arg_in[256];
    char* pch;

    std::string arg = " ";
    std::string remotename;

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "NUMBER", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            _number = std::stoi(arg, nullptr, 10);
            if (_number == 0) {
                std::cout << "%%setSignalInput - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "MODE", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            mode = std::stoi(arg, nullptr, 10);
            if (mode == 0) {
                std::cout << "%%setSignalInput - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "INDEX", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            index = std::stoi(arg, nullptr, 10);
            if (index == 0) {
                std::cout << "%%setSignalInput - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "NAME", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            remotename = std::string(pch);
            std::transform(remotename.begin(), remotename.end(), remotename.begin(), ::toupper);
        }

        pch = strtok(NULL, " ");
    }
 
    if ((_number != 0) && ( ((mode != 0) && (mode < 5) && (index != 0)) || ((mode > 4) && (mode < 9) && (remotename.length() != 0)) )) {
        TheConfig.setSignalInput(_number, mode, index, remotename);
    }
    else {
        std::cout << "%%setSignalInput - syntax error" << std::endl;
        return;
    }

}

void setSignalLogic() {
    int _signal = 0;
    int aspect = 0;
    int NOR[64];
    int nnor = 0;
    char arg_in[256];
    char* pch;

    std::string arg = " ";
    std::string aspectname;

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "SIGNAL", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            _signal = std::stoi(arg, nullptr, 10);
            if (_signal == 0) {
                std::cout << "%%setSignalLogic - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "ASPECT", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            aspectname = std::string(pch);
            std::transform(aspectname.begin(), aspectname.end(), aspectname.begin(), ::toupper);
            if (aspectname.compare(0, 2, "FLASHING", 0, 2) == 0) {
                pch = strtok(NULL, " ");
                std::string aspectname2 = std::string(pch);
                std::transform(aspectname2.begin(), aspectname2.end(), aspectname2.begin(), ::toupper);
                aspectname.append(" ");
                aspectname.append(aspectname2);
            }
            aspect = getAspectFromName(aspectname);
            if (aspect == 0) {
                std::cout << "%%setSignalLogic - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "ON", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            NOR[nnor] = std::stoi(arg, nullptr, 10);
            nnor++;
            if (NOR[nnor-1] == 0) {
                std::cout << "%%setSignalLogic - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "OFF", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            NOR[nnor] = std::stoi(arg, nullptr, 10)+64;
            nnor++;
            if (NOR[nnor-1] == 0) {
                std::cout << "%%setSignalLogic - syntax error" << std::endl;
                return;
            }
        }

        pch = strtok(NULL, " ");
    }

    if ((_signal != 0) && (aspect != 0) && (nnor != 0)) {
        TheConfig.setSignalLogic(_signal,aspect,nnor,NOR);
    }
    else {
        std::cout << "%%setSignalLogic - syntax error" << std::endl;
        return;
    }

}


void setIndicator() {
    int board = 0;
    int channel = 0;
    int _switch = -1;
    int _sensor = -1;
    char arg_in[256];
    char* pch;

    std::string arg = " ";

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setIndicator - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "CHANNEL", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setIndicator - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "SWITCH", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            _switch = std::stoi(arg, nullptr, 10);
            if (_switch == -1) {
                std::cout << "%%setIndicator - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "SENSOR", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            _sensor = std::stoi(arg, nullptr, 10);
            if (_sensor == -1) {
                std::cout << "%%setIndicator - syntax error" << std::endl;
                return;
            }
        }
        pch = strtok(NULL, " ");
    }
    if ((board != 0) && (channel != 0) && !((_switch != 0) && (_sensor != 0)) &&
        (channel > 0) && (channel < 5) && (board > 0) && (board < 3)) {
        TheConfig.setIndicator(4 * (board - 1) + channel - 1, _switch, _sensor);
    }
    else {
        std::cout << "%%setIndicator - syntax error" << std::endl;
        return;
    }

}

void setKeyboard() {
    int track = -1;
    int outputs = 0;
    int clear = 0;
    int output[16];
    char arg_in[256];
    char* pch;

    std::string arg = " ";
    for (int i = 0; i < 16; i++) {
        output[i] = 0;
    }
    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "TRACK", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            track = std::stoi(arg, nullptr, 10);
            if (track == -1) {
                std::cout << "%%setKeyboard - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "ON", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            int index = std::stoi(arg, nullptr, 10);
            if (index != 0) {
                output[index - 1] = 1;
                outputs++;
            }
            if (index == 0) {
                std::cout << "%%setKeyboard - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "CLEAR", 0, 2) == 0) {
            clear = 1;
        }

        pch = strtok(NULL, " ");
    }

    if (clear == 1) {
        for (int i = 0; i < 16; i++) {
            output[i] = 0;
        }
    }
    if ((track != -1) && ((outputs != 0) || (clear == 1))) {
        TheConfig.setKeyboard(track, output);
    }
}

void setDirectory() {
    char arg_in[256];
    std::string arg;
//    std::cin >> arg;
    std::cin.getline(arg_in,256);
    arg = std::string(arg_in);

    std::string oldWD = TheConfig.getWorkingDir();
    arg = arg.substr(1);
    std::cout << arg.substr(1, 1) << std::endl;

    if ( strcmp((arg.substr(1,1)).c_str(),":") == 0 ) {
 //   if (arg.compare(2, 1, ":", 1, 1) == 0) {
        // there is a device declaration in the new working directory
        TheConfig.setWorkingDir(arg);
        writeConfFile();
    }
    else {
        // there isn't a device declaration in the new working directory
        std::string newWD;
        if ( strcmp((arg.substr(0,1)).c_str(),"\\") == 0 ) {
  //       if (arg.compare(1, 1, "\\") == 0) {
            newWD = oldWD.substr(0, 2) + arg;
            TheConfig.setWorkingDir(newWD);
            writeConfFile();
        }
        else {
            std::cout << "%%setDirectory - syntax error" << std::endl;
        }
    }
}


void deleteTurnout() {
    int board = 0;
    int channel = 0;
    char arg_in[256];
    char* pch;

    std::string arg = " ";

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%deleteTurnout - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "CHANNEL", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%deleteTurnout - syntax error" << std::endl;
                return;
            }
        }

        pch = strtok(NULL, " ");
    }
    if ((board != 0) && (channel != 0) &&
        (channel > 0) && (channel < 5) && (board > 0) && (board < 5)) {
        TheConfig.setQuadTurnout(4 * (board - 1) + channel - 1, 0, "");
    }
    else {
        std::cout << "%%deleteTurnout - syntax error" << std::endl;
        return;
    }

}

void deleteQsensor() {
    int board = 0;
    int channel = 0;
    char arg_in[256];
    char* pch;

    std::string arg = " ";

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);

        channel = std::stoi(arg, nullptr, 10);
        if (channel == 0) {
            std::cout << "%%deleteQsensor - syntax error" << std::endl;
            return;
        }
        pch = strtok(NULL, " ");
    }
    if (channel != 0) {
        TheConfig.deleteQuadSensor(channel);
    }
    else {
        std::cout << "%%deleteQsensor - syntax error" << std::endl;
        return;
    }

}

void deleteSensor() {
    int board = 0;
    int channel = 0;
    char arg_in[256];
    char* pch;

    std::string arg = " ";

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%deleteSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "CHANNEL", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%deleteSensor - syntax error" << std::endl;
                return;
            }
        }

        pch = strtok(NULL, " ");
    }
    if ((board != 0) && (channel != 0) &&
        (channel > 0) && (channel < 5) && (board > 0) && (board < 3)) {
        TheConfig.setSensor(board, channel, "");
    }
    else {
        std::cout << "%%deleteSensor - syntax error" << std::endl;
        return;
    }

}

void deleteSignal() {
    int board = 0;
    int channel = 0;
    char arg_in[256];
    char* pch;

    std::string arg = " ";

    std::cin.getline(arg_in, 256);

    pch = strtok(arg_in, " ");
    while (pch != NULL) {
        arg = std::string(pch);
        std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

        if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%deleteSignal - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "CHANNEL", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%deleteSignal - syntax error" << std::endl;
                return;
            }
        }

        pch = strtok(NULL, " ");
    }
    if ((board != 0) && (channel != 0) &&
        (channel > 0) && (channel < 5) && (board > 0) && (board < 3)) {
        TheConfig.deleteSignal(board, channel);
    }
    else {
        std::cout << "%%deleteSignal - syntax error" << std::endl;
        return;
    }

}


void setDataDispatcher() {
    std::string arg;
    std::cin >> arg;
    std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

    if (arg.compare(0, 2, "MAC", 0, 2) == 0) {
        setMacAddr();
    }
    else if (arg.compare(0, 2, "IP", 0, 2) == 0) {
        setIpAddr();
    }
    else if (arg.compare(0, 3, "SERVER", 0, 3) == 0) {
        setServerIpAddr();
    }
    else if (arg.compare(0, 2, "BOARD", 0, 2) == 0) {
        setBoard();
    }
    else if (arg.compare(0, 2, "TURNOUT", 0, 2) == 0) {
        setTurnout();
    }
    else if (arg.compare(0, 2, "QSENSORS", 0, 2) == 0) {
        setQuadSensor();
    }
    else if (arg.compare(0, 2, "DUAL", 0, 2) == 0) {
//        TheConfig.printDualTurnouts();
    }
    else if (arg.compare(0, 3, "SENSORS", 0, 3) == 0) {
        setSensor();
    }
    else if (arg.compare(0, 2, "SIGNALS", 0, 2) == 0) {
        setSignal();
    }
    else if (arg.compare(0, 3, "INPUTS", 0, 3) == 0) {
        setSignalInput();
    }
    else if (arg.compare(0, 2, "LOGIC", 0, 2) == 0) {
        setSignalLogic();
    }
    else if (arg.compare(0, 3, "INDICATORS", 0, 3) == 0) {
        setIndicator();
    }
    else if (arg.compare(0, 2, "KEYBOARD", 0, 2) == 0) {
        setKeyboard();
    }
    else if (arg.compare(0, 2, "DIRECTORY", 0, 2) == 0) {
        setDirectory();
    }
    else {
        std::cout << "%%set - syntax error." << std::endl;
    }
}

void deleteDataDispatcher() {
    std::string arg;
    std::cin >> arg;
    std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);

    if (arg.compare(0, 2, "TURNOUT", 0, 2) == 0) {
        deleteTurnout();
    }
    else if (arg.compare(0, 2, "QSENSOR", 0, 2) == 0) {
        deleteQsensor();
    }
    else if (arg.compare(0, 2, "SENSOR", 0, 2) == 0) {
        deleteSensor();
    }
    else if (arg.compare(0, 2, "SIGNAL", 0, 2) == 0) {
        deleteSignal();
    }
}

void printHelp() {
    std::cout << "Commands Available:" << std::endl;
    std::cout << "  open <filename> - reads file in" << std::endl;
    std::cout << "  show [config|mac|ip|server|boards|turnout|qsensors|dualturnouts|sensors|signals|inputs|logic|indicators|keyboard]" << std::endl;
    std::cout << "  set mac xx-xx-xx-xx-xx-xx - Set the MAC address of the hardware" << std::endl;
    std::cout << "  set ip xxx.xxx.xxx.xxx - Set the IP address of the station" << std::endl;
    std::cout << "  set server xxx.xxx.xxx.xxx - Set the IP address of the JMRI Server to connect on port 2048" << std::endl;
    std::cout << "  set boards [toggle|sensor|turnout|indicator|dual|signal|relay] xx - set number of active board types" << std::endl;
    std::cout << "  set turnout board (1..4) channel (1..4) toggle xx name NTxxxx" << std::endl;
    std::cout << "  set qsensor board (1..4) channel (1..8) sensor (1..16) name ISxxxx" << std::endl;
    std::cout << "  set dual board (1..2) channel (1..2) toggle xx name NTxxxx" << std::endl;
    std::cout << "  set sensor board (1..2) channel (1..4) name ISxxxx" << std::endl;
    std::cout << "  set signal board (1..2) channel (1..4) name NTxxxx head NTxxxx" << std::endl;
    std::cout << "  set inputs number xx mode (1..9) [index xx | name XXxxxx]" << std::endl;
    std::cout << "  set logic signal xx aspect (flashing) [dark|red|lunar|yellow|green] (on xx / off xx)" << std::endl;
    std::cout << "  set indicator board (1..2) channel (1..4) [switch|sensor] xx" << std::endl;
    std::cout << "  set keyboard track (0..15) on (select up to 16 channels)" << std::endl;
    std::cout << "  delete turnout board (1..4) channel (1..4)" << std::endl;
    std::cout << "  delete qsensor xx" << std::endl;
    std::cout << "  delete sensor board (1..2) channel (1..4)" << std::endl;
    std::cout << "  delete signal board (1..2) channel (1..4)" << std::endl;
    std::cout << "  delete input xx" << std::endl;
    std::cout << "  delete logic" << std::endl;

}

int commandLine() {
    static int requestRun = 1;
    std::string cmdIn;
    //    char temp[256];
    while (requestRun == 1) {
        std::cout << "Config> ";
        std::cin >> cmdIn;
        std::transform(cmdIn.begin(), cmdIn.end(), cmdIn.begin(), ::toupper);
        //        std::string cmdRoot = cmdIn.substr(0, cmdIn.find(" "));

        if ((cmdIn.compare("EXIT") == 0) || (cmdIn.compare("QUIT") == 0)) {
            requestRun = 0;
        }
        else if (cmdIn.compare("OPEN") == 0) {
//            std::string fileName;
//            std::cin >> fileName;
//            std::cout << fileName << std::endl;
            openFile();
        }
        else if (cmdIn.compare("SHOW") == 0) {
            showDataDispatcher();
        }
        else if (cmdIn.compare("SET") == 0) {
            setDataDispatcher();
        }
        else if (cmdIn.compare("DELETE") == 0) {
            deleteDataDispatcher();
        }
        else if (cmdIn.compare("HELP") == 0) {
            printHelp();
        }
    }

    return(0);
}

void initConfigFile() {
    std::string dir_name = TheConfig.getConfigDir();
    std::string config_file = dir_name + "\\CMRSconfig.dat";
    std::string working_dir = std::string(getenv("USERPROFILE")) + "\\Desktop";

    std::cout << "Config File = " << config_file << std::endl;
    std::cout << "Working Dir = " << working_dir << std::endl;
    //    std::cout << "initConfigFile " << config_file << std::endl;
    std::ifstream fs;
    fs.open(config_file.c_str(), std::fstream::in);
    if (fs.good()) {
        std::cout << "fs.good" << std::endl;
        char arg_in[256];
        while (!(fs.eof())) {
            fs.getline(arg_in, 256, '\n');
            if (fs.gcount() != 0) {
                std::cout << std::string(arg_in) << std::endl;
                char* pch = strtok(arg_in, " ");
                std::string arg = std::string(pch);
                if (arg.compare(0, 3, "#WD", 0, 3) == 0) {
                    pch = strtok(NULL, " ");
                    TheConfig.setWorkingDir(std::string(pch));
                }
            }
        }
    }
        // get working directory
    else {
        std::cout << "fs no good " << config_file << std::endl;
        fs.close();
        std::ofstream fsout;
        fsout.open(config_file.c_str(), std::fstream::out);
        fsout << "#WD " << working_dir << std::endl;
        fsout.close();
        TheConfig.setWorkingDir(working_dir);
    }

}

int initConfig() {

    char* adir;
    adir = getenv("APPDATA");
    std::string appdatadir = std::string(adir);
    std::string dir_name = appdatadir+"\\CMRS";
    std::cout << "Creating directory " << dir_name << std::endl;
#ifdef _WIN32
    int status = mkdir(dir_name.c_str());
#else
    int status = mkdir(dir_name.c_str(), 0777);
#endif
    TheConfig.setConfigDir(dir_name);

    if (status == 0) {
        std::cout << "Directory created successfully." << std::endl;
    }
    else {
        std::cout << "Error creating directory: ";
        switch (errno) {
        case EEXIST:
            std::cout << "Directory already exists." << std::endl;
            break;
        case ENOENT:
            std::cout << "Parent directory does not exist." << std::endl;
            break;
        default:
            std::cout << "Unknown error." << std::endl;
        }
        return 1;
    }
    return 0;
}

int main(int argc, char* argv[])
{
    std::cout << "Number of arguments " << argc << std::endl;
    
 //   TheConfig = new CMRSconfig();
    initConfig();
    initConfigFile();

  //  getData();

    commandLine();

    std::cout << "Exiting...";



 //   std::string cmdIn;
 //   std::cin >> cmdIn;

}
