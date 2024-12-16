#include "CMRSconfig.h"

#include <iostream>


CMRSconfig::CMRSconfig()
{
	int i;
	for (i = 0; i < 6; i++) {
		macaddr[i] = 0;
		boards[i] = 0;
	}
	for (i = 0; i < 4; i++) {
		ipaddr[i] = 0;
		serveripaddr[i] = 0;
	}
	for (i = 0; i < 16; i++) {
		quadpos[i] = 0;
		power[i] = 0;
	}
	for (i = 0; i < 8; i++) {
		dualpos[i] = 0;
	}
	for (i = 0; i < 64; i++) {
		norData[i] = 0;
	}
}

CMRSconfig::~CMRSconfig()
{

}

void CMRSconfig::parse_data(const std::string& arg)
{
	size_t n1 = arg.find(" ");
	int index = stoi(arg.substr(0, n1));
	int val = stoi(arg.substr(n1, std::string::npos));
//	std::cout << index << "////" << val << std::endl;

	if (index < 6) {
		macaddr[index] = val;
	}
	else if (index < 8) {
		// NOP
	}
	else if (index < 12) {
		ipaddr[index - 8] = val;
	}
	else if (index < 16) {
		serveripaddr[index - 12] = val;
	}
	else if (index < 23) {
		boards[index - 16] = val;
	}
	else if (index < 40) {
		//NOP
	}
	else if (index < 56) {
		quadpos[index - 40] = val;
	}
	else if (index < 64) {
		dualpos[index - 56] = val;
	}
	else if (index < 192) {
		int sindex = index - 64;
		int turnoutNumber = sindex / 8;
		int turnoutByte = sindex % 8;
		if (turnoutByte == 0) {
			quadTurnout[turnoutNumber].set_toggle(val);
			byteDone = 0;
		}
		else {
			if ((val != 0) && (byteDone == 0)) {
				quadTurnout[turnoutNumber].add_name(val);
			}
			else {
				byteDone = 1;
			}
		}
	}
	else if (index < 352) {
		int sindex = index - 192;
		int iNumber = sindex / 10;
		int iByte = sindex % 10;
		if (iByte == 0) {
			quadSensor[iNumber].set_board(val);
			byteDone = 0;
		}
		else if (iByte == 1) {
			quadSensor[iNumber].set_sensor(val);
		}
		else if (iByte == 9) {
			quadSensor[iNumber].set_sensornum(val);
		}
		else {
			if ((val != 0) && (byteDone == 0)) {
				quadSensor[iNumber].add_name(val);
			}
			else {
				byteDone = 1;
			}
		}
	}
	else if (index < 416) {
		int sindex = index - 352;
		int turnoutNumber = sindex / 8;
		int turnoutByte = sindex % 8;
		if (turnoutByte == 0) {
			dualTurnout[turnoutNumber].set_toggle(val);
			byteDone = 0;
		}
		else {
			if ((val != 0) && (byteDone == 0)) {
				dualTurnout[turnoutNumber].add_name(val);
			}
			else {
				byteDone = 1;
			}
		}
	}
	else if (index < 416) {
		int sindex = index - 352;
		int turnoutNumber = sindex / 8;
		int turnoutByte = sindex % 8;
		if (turnoutByte == 0) {
			dualTurnout[turnoutNumber].set_toggle(val);
			byteDone = 0;
		}
		else {
			if ((val != 0) && (byteDone == 0)) {
				dualTurnout[turnoutNumber].add_name(val);
			}
			else {
				byteDone = 1;
			}
		}
	}
	else if (index < 544) {
		int sindex = index - 416;
		int iNumber = sindex / 8;
		int iByte = sindex % 8;
		if (iByte == 0) {
			sensor[iNumber].set_toggle(val);  // using toggle as the active flag for sensor
			byteDone = 0;
		}
		else {
			if ((val != 0) && (byteDone == 0)) {
				sensor[iNumber].add_name(val);
			}
			else {
				byteDone = 1;
			}
		}
	}
	else if (index < 832) {
		int sindex = index - 544;
		int iNumber = sindex / 9;
		int iByte = sindex % 9;
		if (iByte == 0) {
			signalInput[iNumber].set_mode(val);
			byteDone = 0;
		}
		else if (iByte == 1) {
			signalInput[iNumber].set_index(val);
		}
		else {
			if ((val != 0) && (byteDone == 0)) {
				signalInput[iNumber].add_name(val);
			}
			else {
				byteDone = 1;
			}
		}
	}
	else if (index < 896) {
		int sindex = index - 832;
		int iNumber = sindex / 4;
		int iByte = sindex % 4;
		switch (iByte) {
			case 0 :
				signalLogic[iNumber].set_signal(val);
				break;
			case 1:
				signalLogic[iNumber].set_aspect(val);
				break;
			case 2:
				signalLogic[iNumber].set_firstNOR(val);
				break;
			case 3:
				signalLogic[iNumber].set_numNOR(val);
				break;
		}
	}
	else if (index < 928) {
		// NOP
	}
	else if (index < 944) {
		int sindex = index - 928;
		int iNumber = sindex / 2;
		int iByte = sindex % 2;
		if (iByte == 0) {
			indicator[iNumber].set_switch(val);
		}
		else {
			indicator[iNumber].set_sensor(val);
		}
	}
	else if (index < 960) {
		// NOP
	}
	else if (index < 976) {
		power[index - 960] = val;
	}
	else if (index < 1024) {
		// NOP
	}
	else if (index < 1280) {
		int sindex = index - 1024;
		int iTrack = sindex / 16;
		int iSwitch = sindex % 16;
		kbTrack[iTrack].set_mode(iSwitch, val);
	}
	else if (index < 1300) {
		// NOP
	}
	else if (index < 1540) {
		int sindex = index - 1300;
		int iNumber = sindex / 15;
		int iByte = sindex % 15;
		if (iByte == 0) {
			signal[iNumber].set_active(val);
			byteDone = 0;
		}
		else if (iByte < 8) {
			if ((val != 0) && (byteDone == 0)) {
				signal[iNumber].add_signalhead(val);
			}
			else {
				byteDone = 1;
			}
		}
		else {
			if (iByte == 8) {
				byteDone = 0;
			}
			if ((val != 0) && (byteDone == 0)) {
				signal[iNumber].add_leadingsignal(val);
			}
			else {
				byteDone = 1;
			}
		}
	}
	else if (index < 1604) {
		int sindex = index - 1540;
		for (int i = 0; i < 16; i++) {
			if (signalLogic[i].get_signal() != 0) {
				int first = signalLogic[i].get_firstNOR();
				int num = signalLogic[i].get_numNOR();
				if ((first <= sindex) && (sindex < (first + num))) {
					signalLogic[i].add_NOR(sindex - first, val);
				}
			}
		}
	}
	else {
		// NOP
	}
}

void CMRSconfig::printMacAddr() {
	std::cout << std::hex;
	std::cout << "Mac Address ";
	for (int i = 0; i < 5; i++) {
		std::cout << macaddr[i] << " - ";
	}
	std::cout << macaddr[5] << std::dec << std::endl;
}

void CMRSconfig::printIpAddr() {
	std::cout << "IP Address  ";
	for (int i = 0; i < 3; i++) {
		std::cout << ipaddr[i] << ".";
	}
	std::cout << ipaddr[3] << std::endl;
}

void CMRSconfig::printServerIpAddr() {
	std::cout << "Server IP   ";
	for (int i = 0; i < 3; i++) {
		std::cout << serveripaddr[i] << ".";
	}
	std::cout << serveripaddr[3] << std::endl;
}

void CMRSconfig::printBoards() {
	std::cout << "Boards: ";
	std::cout << "Toggle " << boards[0];
	std::cout << " Sensor " << boards[1];
	std::cout << " Quad " << boards[2];
	std::cout << " Indicator " << boards[3];
	std::cout << " Signal " << boards[4];
	std::cout << " Dual " << boards[5];
	std::cout << " Relay " << boards[6] << std::endl;
}

void CMRSconfig::printQuadTurnouts() {
	std::cout << "Quad Turnouts:" << std::endl;
	for (int i = 0; i < 16; i++) {
		if (quadTurnout[i].get_toggle() != 0) {
			std::cout << "  Turnout " << (i / 4) + 1 << "/" << (i % 4) + 1;
			std::cout << " Toggle " << quadTurnout[i].get_toggle() << " Name ";
			std::cout << quadTurnout[i].get_name() << std::endl;
		}
	}
}

void CMRSconfig::printQuadSensors() {
	std::cout << "Quad Sensors:" << std::endl;
	for (int i = 0; i < 16; i++) {
		if ((quadSensor[i].get_board() != 0) && (quadSensor[i].get_sensor() != 0)) {
			std::cout << "  Sensor " << i << " Board " << quadSensor[i].get_board();
			std::cout << " Sensor " << quadSensor[i].get_sensor() << " Name " << quadSensor[i].get_name();
			std::cout << " Sensor # " << quadSensor[i].get_sensornum() << std::endl;
		}
	}
}

void CMRSconfig::printDualTurnouts() {
	if (boards[5] != 0) {
		std::cout << "Dual Turnouts:" << std::endl;
		for (int i = 0; i < 8; i++) {
			if (dualTurnout[i].get_toggle() != 0) {
				std::cout << "  Turnout " << (i / 2) + 1 << "/" << (i % 2) + 1;
				std::cout << " Toggle " << dualTurnout[i].get_toggle() << " Name ";
				std::cout << dualTurnout[i].get_name() << std::endl;
			}
		}
	}
}

void CMRSconfig::printSensors() {
	std::cout << "Sensors:" << std::endl;
	for (int i = 0; i < 16; i++) {
		if (sensor[i].get_toggle() != 0) {
			std::cout << "  Sensor " << i << " Name " << sensor[i].get_name() << std::endl;
		}
	}
}

void CMRSconfig::printSignalInputs() {
	std::cout << "Signal Inputs:" << std::endl;
	for (int i = 0; i < 32; i++) {
		if (signalInput[i].get_mode() != 0) {
			std::cout << "  Input " << i << " Mode " << signalInput[i].get_mode();
			std::cout << " Index " << signalInput[i].get_index();
			std::cout << " Name " << signalInput[i].get_name() << std::endl;
		}
	}
}

void CMRSconfig::printSignalLogic() {
	std::cout << "Signal Logic:" << std::endl;
	for (int i = 0; i < 16; i++) {
		if (signalLogic[i].get_signal() != 0) {
			std::cout << "  " << i << " Signal " << signalLogic[i].get_signal();
			std::cout << " Aspect " << signalLogic[i].get_aspect();
			std::cout << " " << getSignalAspect(signalLogic[i].get_aspect());
			std::cout << " NORs " << signalLogic[i].get_firstNOR() << "/" << signalLogic[i].get_numNOR() << " : ";
			for (int j = 0; j < signalLogic[i].get_numNOR(); j++) {
				std::cout << signalLogic[i].get_NOR(j) << " ";
			}
			std::cout << std::endl;

			std::cout << "    " << signal[signalLogic[i].get_signal() - 1].get_signalhead() << " ";
			std::cout << getSignalAspect(signalLogic[i].get_aspect()) << " == ";
			for (int j = 0; j < signalLogic[i].get_numNOR(); j++) {
				int tNOR = signalLogic[i].get_NOR(j);
				if (tNOR < 64) {
					std::cout << " !(Input " << tNOR << ") ";
				}
				else {
					std::cout << " (Input " << tNOR - 64 << ") ";
				}
				if (j < signalLogic[i].get_numNOR() - 1) {
					std::cout << "&&";
				}
			}
			std::cout << std::endl;
		}
	}
}

void CMRSconfig::printIndicators() {
	std::cout << "Indicators:" << std::endl;
	for (int i = 0; i < 8; i++) {
		std::cout << " Indicator " << (i / 4) + 1 << "/" << (i % 4) + 1;
		std::cout << " Switch " << indicator[i].get_switch() << " Sensor " << indicator[i].get_sensor() << std::endl;
	}
}

void CMRSconfig::printKBTrack() {
	if (boards[6] != 0) {
		std::cout << "Keyboard Track Map:" << std::endl;
		for (int i = 0; i < 16; i++) {
			std::cout << "  Track " << i << " : ";
			for (int j = 0; j < 16; j++) {
				std::cout << kbTrack[i].get_mode(j) << " ";
			}
			std::cout << std::endl;
		}
	}
}

void CMRSconfig::printSignals() {
	std::cout << "Signals:" << std::endl;
	for (int i = 0; i < 16; i++) {
		if (signal[i].get_active() != 0) {
			std::cout << "  Signal " << signal[i].get_active() << " Name " << signal[i].get_signalhead();
			std::cout << " Lead Signal " << signal[i].get_leadingsignal() << std::endl;
		}
	}
}