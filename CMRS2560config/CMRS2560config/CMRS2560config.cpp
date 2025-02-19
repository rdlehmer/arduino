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

int getAspectFromName(std::string arg_name) {
    int ret_val = 0;
    if (arg_name.compare("dark") == 0) {
        ret_val = 1;
    }
    else if (arg_name.compare("red") == 0) {
        ret_val = 2;
    }
    else if (arg_name.compare("flashing red") == 0) {
        ret_val = 3;
    }
    else if (arg_name.compare("flunar") == 0) {
        ret_val = 4;
    }
    else if (arg_name.compare("flashing lunar") == 0) {
        ret_val = 5;
    }
    else if (arg_name.compare("yellow") == 0) {
        ret_val = 6;
    }
    else if (arg_name.compare("flashing yellow") == 0) {
        ret_val = 7;
    }
    else if (arg_name.compare("green") == 0) {
        ret_val = 8;
    }
    else if (arg_name.compare("flashing green") == 0) {
        ret_val = 9;
    }
    return(ret_val);
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
        if (arg.compare(0, 2, "board", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setTurnout - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "channel", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch); 
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setTurnout - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "name", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            turnoutname = std::string(pch);
        }

        if (arg.compare(0, 2, "toggle", 0, 2) == 0) {
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
        if (arg.compare(0, 2, "board", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setQuadSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "channel", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setQuadSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "name", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            sensorname = std::string(pch);
        }

        if (arg.compare(0, 2, "sensor", 0, 2) == 0) {
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
        if (arg.compare(0, 2, "board", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "channel", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "name", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            sensorname = std::string(pch);
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
        if (arg.compare(0, 2, "board", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setSignal - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "channel", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setSignal - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "name", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            signalname = std::string(pch);
        }

        if (arg.compare(0, 2, "lead", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            leadingsignalname = std::string(pch);
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
        if (arg.compare(0, 2, "number", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            _number = std::stoi(arg, nullptr, 10);
            if (_number == 0) {
                std::cout << "%%setSignalInput - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "mode", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            mode = std::stoi(arg, nullptr, 10);
            if (mode == 0) {
                std::cout << "%%setSignalInput - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "index", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            index = std::stoi(arg, nullptr, 10);
            if (index == 0) {
                std::cout << "%%setSignalInput - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "name", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            remotename = std::string(pch);
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
        if (arg.compare(0, 2, "signal", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            _signal = std::stoi(arg, nullptr, 10);
            if (_signal == 0) {
                std::cout << "%%setSignalLogic - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "aspect", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            aspectname = std::string(pch);
            if (aspectname.compare(0, 2, "flashing", 0, 2) == 0) {
                pch = strtok(NULL, " ");
                aspectname.append(" ");
                aspectname.append(std::string(pch));
            }
            aspect = getAspectFromName(aspectname);
            if (aspect == 0) {
                std::cout << "%%setSignalLogic - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "on", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            NOR[nnor] = std::stoi(arg, nullptr, 10);
            nnor++;
            if (NOR[nnor-1] == 0) {
                std::cout << "%%setSignalLogic - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "off", 0, 2) == 0) {
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
        if (arg.compare(0, 2, "board", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%setIndicator - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "channel", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            channel = std::stoi(arg, nullptr, 10);
            if (channel == 0) {
                std::cout << "%%setIndicator - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "switch", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            _switch = std::stoi(arg, nullptr, 10);
            if (_switch == -1) {
                std::cout << "%%setIndicator - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "sensor", 0, 2) == 0) {
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
        if (arg.compare(0, 2, "track", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            track = std::stoi(arg, nullptr, 10);
            if (track == -1) {
                std::cout << "%%setKeyboard - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "on", 0, 2) == 0) {
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

        if (arg.compare(0, 2, "clear", 0, 2) == 0) {
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
        if (arg.compare(0, 2, "board", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%deleteTurnout - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "channel", 0, 2) == 0) {
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
        if (arg.compare(0, 2, "board", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%deleteSensor - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "channel", 0, 2) == 0) {
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
        if (arg.compare(0, 2, "board", 0, 2) == 0) {
            pch = strtok(NULL, " ");
            arg = std::string(pch);
            board = std::stoi(arg, nullptr, 10);
            if (board == 0) {
                std::cout << "%%deleteSignal - syntax error" << std::endl;
                return;
            }
        }

        if (arg.compare(0, 2, "channel", 0, 2) == 0) {
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
    }
    else if (arg.compare(0, 2, "qsensors", 0, 2) == 0) {
        setQuadSensor();
    }
    else if (arg.compare(0, 2, "dual", 0, 2) == 0) {
//        TheConfig.printDualTurnouts();
    }
    else if (arg.compare(0, 3, "sensors", 0, 3) == 0) {
        setSensor();
    }
    else if (arg.compare(0, 2, "signals", 0, 2) == 0) {
        setSignal();
    }
    else if (arg.compare(0, 3, "inputs", 0, 3) == 0) {
        setSignalInput();
    }
    else if (arg.compare(0, 2, "logic", 0, 2) == 0) {
        setSignalLogic();
    }
    else if (arg.compare(0, 3, "indicators", 0, 3) == 0) {
        setIndicator();
    }
    else if (arg.compare(0, 2, "keyboard", 0, 2) == 0) {
        setKeyboard();
    }
    else {
        std::cout << "%%set - syntax error." << std::endl;
    }
}

void deleteDataDispatcher() {
    std::string arg;
    std::cin >> arg;

    if (arg.compare(0, 2, "turnout", 0, 2) == 0) {
        deleteTurnout();
    }
    else if (arg.compare(0, 2, "qsensor", 0, 2) == 0) {
        deleteQsensor();
    }
    else if (arg.compare(0, 2, "sensor", 0, 2) == 0) {
        deleteSensor();
    }
    else if (arg.compare(0, 2, "signal", 0, 2) == 0) {
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
    std::cout << "  set inputs " << std::endl;
    std::cout << "  set logic " << std::endl;
    std::cout << "  set indicator board (1..2) channel (1..4) [switch|sensor] xx" << std::endl;
    std::cout << "  set keyboard track (0..15) on (select up to 16 channels)" << std::endl;
    std::cout << "  delete turnout board (1..4) channel (1..4)" << std::endl;
    std::cout << "  delete qsensor xx" << std::endl;
    std::cout << "  delete sensor board (1..2) channel (1..4)" << std::endl;
    std::cout << "  delete signal board (1..2) channel (1..4)" << std::endl;

}

int commandLine() {
    static int requestRun = 1;
    std::string cmdIn;
//    char temp[256];
    while (requestRun == 1) {
        std::cout << "Config> ";
        std::cin >> cmdIn;

        std::string cmdRoot = cmdIn.substr(0, cmdIn.find(" "));

        if ((cmdIn.compare("exit") == 0) || (cmdIn.compare("quit") == 0)) {
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
        else if (cmdIn.compare("delete") == 0) {
            deleteDataDispatcher();
        }
        else if (cmdIn.compare("help") == 0) {
            printHelp();
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
