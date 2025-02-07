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

	if (index < CMRS_MACADDR_OFFSET+CMRS_MACADDR_SIZE) {
		macaddr[index] = val;
	}
	else if (index < CMRS_IPADDR_OFFSET) {
		// NOP
	}
	else if (index < CMRS_IPADDR_OFFSET+CMRS_IPADDR_SIZE) {
		ipaddr[index - CMRS_IPADDR_OFFSET] = val;
	}
	else if (index < CMRS_SIPADDR_OFFSET+CMRS_SIPADDR_SIZE) {
		serveripaddr[index - CMRS_SIPADDR_OFFSET] = val;
	}
	else if (index < CMRS_BOARDS_OFFSET+CMRS_BOARDS_SIZE) {
		boards[index - CMRS_BOARDS_OFFSET] = val;
	}
	else if (index < CMRS_QUADPOS_OFFSET) {
		//NOP
	}
	else if (index < CMRS_QUADPOS_OFFSET+CMRS_QUADPOS_SIZE) {
		quadpos[index - CMRS_QUADPOS_OFFSET] = val;
	}
	else if (index < CMRS_DUALPOS_OFFSET+CMRS_DUALPOS_SIZE) {
		dualpos[index - CMRS_DUALPOS_OFFSET] = val;
	}
	else if (index < CMRS_QUADTURNOUT_OFFSET+CMRS_TURNOUT_SIZE*CMRS_QUADTURNOUT_NUMBER) {
		int sindex = index - CMRS_QUADTURNOUT_OFFSET;
		int turnoutNumber = sindex / CMRS_TURNOUT_SIZE;
		int turnoutByte = sindex % CMRS_TURNOUT_SIZE;
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
	else if (index < CMRS_QUADSENSOR_OFFSET+CMRS_QUADSENSOR_SIZE*CMRS_QUADSENSOR_NUMBER) {
		int sindex = index - CMRS_QUADSENSOR_OFFSET;
		int iNumber = sindex / CMRS_QUADSENSOR_SIZE;
		int iByte = sindex % CMRS_QUADSENSOR_SIZE;
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
	else if (index < CMRS_DUALTURNOUT_OFFSET+CMRS_TURNOUT_SIZE*CMRS_DUALTURNOUT_NUMBER) {
		int sindex = index - CMRS_DUALTURNOUT_OFFSET;
		int turnoutNumber = sindex / CMRS_TURNOUT_SIZE;
		int turnoutByte = sindex % CMRS_TURNOUT_SIZE;
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
	else if (index < CMRS_SENSOR_OFFSET+CMRS_SENSOR_SIZE*CMRS_SENSOR_NUMBER) {
		int sindex = index - CMRS_SENSOR_OFFSET;
		int iNumber = sindex / CMRS_SENSOR_SIZE;
		int iByte = sindex % CMRS_SENSOR_SIZE;
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
	else if (index < CMRS_SIGNALINPUT_OFFSET+CMRS_SIGNALINPUT_SIZE*CMRS_SIGNALINPUT_NUMBER) {
		int sindex = index - CMRS_SIGNALINPUT_OFFSET;
		int iNumber = sindex / CMRS_SIGNALINPUT_SIZE;
		int iByte = sindex % CMRS_SIGNALINPUT_SIZE;
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
	else if (index < CMRS_SIGNALLOGIC_OFFSET+CMRS_SIGNALLOGIC_SIZE*CMRS_SIGNALLOGIC_NUMBER) {
		int sindex = index - CMRS_SIGNALLOGIC_OFFSET;
		int iNumber = sindex / CMRS_SIGNALLOGIC_SIZE;
		int iByte = sindex % CMRS_SIGNALLOGIC_SIZE;
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
	else if (index < CMRS_INDICATORS_OFFSET) {
		// NOP
	}
	else if (index < CMRS_INDICATORS_OFFSET+CMRS_INDICATORS_SIZE*CMRS_INDICATORS_NUMBER) {
		int sindex = index - CMRS_INDICATORS_OFFSET;
		int iNumber = sindex / CMRS_INDICATORS_SIZE;
		int iByte = sindex % CMRS_INDICATORS_SIZE;
		if (iByte == 0) {
			indicator[iNumber].set_switch(val);
		}
		else {
			indicator[iNumber].set_sensor(val);
		}
	}
	else if (index < CMRS_POWER_OFFSET) {
		// NOP
	}
	else if (index < CMRS_POWER_OFFSET + 16) {
		power[index - CMRS_POWER_OFFSET] = val;
	}
	else if (index < CMRS_KBTRACK_OFFSET) {
		// NOP
	}
	else if (index < CMRS_KBTRACK_OFFSET + 256) {
		int sindex = index - CMRS_KBTRACK_OFFSET;
		int iTrack = sindex / 16;
		int iSwitch = sindex % 16;
		kbTrack[iTrack].set_mode(iSwitch, val);
	}
	else if (index < CMRS_SIGNAL_OFFSET) {
		// NOP
	}
	else if (index < CMRS_SIGNAL_OFFSET+CMRS_SIGNAL_SIZE*CMRS_SIGNAL_NUMBER) {
		int sindex = index - CMRS_SIGNAL_OFFSET;
		int iNumber = sindex / CMRS_SIGNAL_SIZE;
		int iByte = sindex % CMRS_SIGNAL_SIZE;
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

void CMRSconfig::writePad(int ioffset, int isize) {
	for (int i = 0; i < isize; i++) {
		outfile << (ioffset + i) << " 0" << std::endl;
	}
}

void CMRSconfig::printMacAddr() {
	std::cout << std::hex;
	std::cout << "Mac Address ";
	for (int i = 0; i < CMRS_MACADDR_SIZE-1; i++) {
		std::cout << macaddr[i] << "-";
	}
	std::cout << macaddr[CMRS_MACADDR_SIZE-1] << std::dec << std::endl;
}

void CMRSconfig::writeMacAddr() {
	for (int i = 0; i < CMRS_MACADDR_SIZE; i++) {
		outfile << i << " " << macaddr[i] << std::endl;
	}
}

void CMRSconfig::setMacAddr(int arg[]) {
	for (int i = 0; i < 6; i++) {
		macaddr[i] = arg[i];
	}
}

void CMRSconfig::printIpAddr() {
	std::cout << "IP Address  ";
	for (int i = 0; i < CMRS_IPADDR_SIZE-1; i++) {
		std::cout << ipaddr[i] << ".";
	}
	std::cout << ipaddr[CMRS_IPADDR_SIZE-1] << std::endl;
}

void CMRSconfig::writeIpAddr() {
	for (int i = 0; i < CMRS_IPADDR_SIZE; i++) {
		outfile << (CMRS_IPADDR_OFFSET + i) << " " << ipaddr[i] << std::endl;
	}
}

void CMRSconfig::setIpAddr(int arg[]) {
	for (int i = 0; i < 4; i++) {
		ipaddr[i] = arg[i];
	}
}

void CMRSconfig::printServerIpAddr() {
	std::cout << "Server IP   ";
	for (int i = 0; i < CMRS_SIPADDR_SIZE-1; i++) {
		std::cout << serveripaddr[i] << ".";
	}
	std::cout << serveripaddr[CMRS_SIPADDR_SIZE-1] << std::endl;
}

void CMRSconfig::writeServerIpAddr() {
	for (int i = 0; i < CMRS_SIPADDR_SIZE; i++) {
		outfile << (CMRS_SIPADDR_OFFSET + i) << " " << serveripaddr[i] << std::endl;
	}
}

void CMRSconfig::setServerIpAddr(int arg[]) {
	for (int i = 0; i < 4; i++) {
		serveripaddr[i] = arg[i];
	}
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

void CMRSconfig::writeBoards() {
	for (int i = 0; i < CMRS_BOARDS_SIZE; i++) {
		outfile << (CMRS_BOARDS_OFFSET + i) << " " << boards[i] << std::endl;
	}
}

void CMRSconfig::setBoard(int index, int val) {
	boards[index] = val;
}

void CMRSconfig::writePositions() {
	for (int i = 0; i < CMRS_QUADPOS_SIZE; i++) {
		outfile << (CMRS_QUADPOS_OFFSET + i) << " " << quadpos[i] << std::endl;
	}
	for (int i = 0; i < CMRS_DUALPOS_SIZE; i++) {
		outfile << (CMRS_DUALPOS_OFFSET + i) << " " << dualpos[i] << std::endl;
	}
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

void CMRSconfig::writeQuadTurnouts() {
	int sindex = CMRS_QUADTURNOUT_OFFSET;
	for (int i = 0; i < CMRS_QUADTURNOUT_NUMBER; i++) {
		outfile << sindex << " " << quadTurnout[i].get_toggle() << std::endl;
		sindex++;
		std::string stemp = quadTurnout[i].get_name();
		for (int j = 0; j < CMRS_STRING_SIZE; j++) {
			if (j < stemp.length()) {
				outfile << sindex << " " << int(stemp.at(j)) << std::endl;
			}
			else {
				outfile << sindex << " 0" << std::endl;
			}
			sindex++;
		}
	}
}

void CMRSconfig::setQuadTurnout(int channel, int toggle, std::string turnoutName) {
	quadTurnout[channel].set_toggle(toggle);
	quadTurnout[channel].set_name(turnoutName);
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

void CMRSconfig::writeQuadSensors() {
	int sindex = CMRS_QUADSENSOR_OFFSET;
	for (int i = 0; i < CMRS_QUADSENSOR_NUMBER; i++) {
		outfile << sindex << " " << quadSensor[i].get_board() << std::endl;
		sindex++;
		outfile << sindex << " " << quadSensor[i].get_sensor() << std::endl;
		sindex++;
		std::string stemp = quadSensor[i].get_name();
		for (int j = 0; j < CMRS_STRING_SIZE; j++) {
			if (j < stemp.length()) {
				outfile << sindex << " " << int(stemp.at(j)) << std::endl;
			}
			else {
				outfile << sindex << " 0" << std::endl;
			}
			sindex++;
		}
		outfile << sindex << " " << quadSensor[i].get_sensornum() << std::endl;
		sindex++;
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

void CMRSconfig::writeDualTurnouts() {
	int sindex = CMRS_DUALTURNOUT_OFFSET;
	for (int i = 0; i < CMRS_DUALTURNOUT_NUMBER; i++) {
		outfile << sindex << " " << dualTurnout[i].get_toggle() << std::endl;
		sindex++;
		std::string stemp = dualTurnout[i].get_name();
		for (int j = 0; j < CMRS_STRING_SIZE; j++) {
			if (j < stemp.length()) {
				outfile << sindex << " " << int(stemp.at(j)) << std::endl;
			}
			else {
				outfile << sindex << " 0" << std::endl;
			}
			sindex++;
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

void CMRSconfig::writeSensors() {
	int sindex = CMRS_SENSOR_OFFSET;
	for (int i = 0; i < CMRS_SENSOR_NUMBER; i++) {
		outfile << sindex << " " << sensor[i].get_toggle() << std::endl;
		sindex++;
		std::string stemp = sensor[i].get_name();
		for (int j = 0; j < CMRS_STRING_SIZE; j++) {
			if (j < stemp.length()) {
				outfile << sindex << " " << int(stemp.at(j)) << std::endl;
			}
			else {
				outfile << sindex << " 0" << std::endl;
			}
			sindex++;
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

void CMRSconfig::writeSignalInputs() {
	int sindex = CMRS_SIGNALINPUT_OFFSET;
	for (int i = 0; i < CMRS_SIGNALINPUT_NUMBER; i++) {
		outfile << sindex << " " << signalInput[i].get_mode() << std::endl;
		sindex++;
		outfile << sindex << " " << signalInput[i].get_index() << std::endl;
		sindex++;
		std::string stemp = signalInput[i].get_name();
		for (int j = 0; j < CMRS_STRING_SIZE; j++) {
			if (j < stemp.length()) {
				outfile << sindex << " " << int(stemp.at(j)) << std::endl;
			}
			else {
				outfile << sindex << " 0" << std::endl;
			}
			sindex++;
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

void CMRSconfig::writeSignalLogic() {
	int sindex = CMRS_SIGNALLOGIC_OFFSET;
	for (int i = 0; i < CMRS_SIGNALLOGIC_NUMBER; i++) {
		outfile << sindex << " " << signalLogic[i].get_signal() << std::endl;
		sindex++;
		outfile << sindex << " " << signalLogic[i].get_aspect() << std::endl;
		sindex++;
		outfile << sindex << " " << signalLogic[i].get_firstNOR() << std::endl;
		sindex++;
		outfile << sindex << " " << signalLogic[i].get_numNOR() << std::endl;
		sindex++;
	}
}

void CMRSconfig::printIndicators() {
	std::cout << "Indicators:" << std::endl;
	for (int i = 0; i < 8; i++) {
		std::cout << " Indicator " << (i / 4) + 1 << "/" << (i % 4) + 1;
		std::cout << " Switch " << indicator[i].get_switch() << " Sensor " << indicator[i].get_sensor() << std::endl;
	}
}

void CMRSconfig::writeIndicators() {
	int sindex = CMRS_INDICATORS_OFFSET;
	for (int i = 0; i < CMRS_INDICATORS_NUMBER; i++) {
		outfile << sindex << " " << indicator[i].get_switch() << std::endl;
		sindex++;
		outfile << sindex << " " << indicator[i].get_sensor() << std::endl;
		sindex++;
	}
}

void CMRSconfig::writePower() {
	int sindex = CMRS_POWER_OFFSET;
	for (int i = 0; i < 16; i++) {
		outfile << sindex << " " << power[i] << std::endl;
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

void CMRSconfig::writeKBTrack() {
	int sindex = CMRS_KBTRACK_OFFSET;
	for (int i = 0; i < 16; i++) {
		for (int j = 0; j < 16; j++) {
			outfile << sindex << " " << kbTrack[i].get_mode(j) << std::endl;
			sindex++;
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

void CMRSconfig::writeSignals() {
	int sindex = CMRS_SIGNAL_OFFSET;
	for (int i = 0; i < CMRS_SIGNAL_NUMBER; i++) {
		outfile << sindex << " " << signal[i].get_active() << std::endl;
		sindex++;
		std::string stemp = signal[i].get_signalhead();
		for (int j = 0; j < CMRS_STRING_SIZE; j++) {
			if (j < stemp.length()) {
				outfile << sindex << " " << int(stemp.at(j)) << std::endl;
			}
			else {
				outfile << sindex << " 0" << std::endl;
			}
			sindex++;
		}
		stemp = signal[i].get_leadingsignal();
		for (int j = 0; j < CMRS_STRING_SIZE; j++) {
			if (j < stemp.length()) {
				outfile << sindex << " " << int(stemp.at(j)) << std::endl;
			}
			else {
				outfile << sindex << " 0" << std::endl;
			}
			sindex++;
		}
	}
}

void CMRSconfig::writeNOR() {
	int temp[CMRS_NOR_SIZE];
	int sindex = CMRS_NOR_OFFSET;
	for (int i = 0; i < CMRS_NOR_SIZE; i++) {
		temp[i] = 0;
	}
	for (int i = 0; i < 16; i++) {
		int stemp = signalLogic[i].get_firstNOR();
		for (int j = 0; j < signalLogic[i].get_numNOR(); j++) {
			temp[stemp + j] = signalLogic[i].get_NOR(j);
		}
	}
	for (int i = 0; i < CMRS_NOR_SIZE; i++) {
		outfile << sindex << " " << temp[i] << std::endl;
		sindex++;
	}
}
int CMRSconfig::writeData() {
	outfile.open("..\\..\\..\\CMRS_CP_2560\\station-conf\\64.out", std::ofstream::out);

	if (!(outfile.is_open())) {
		std::perror("Can't open outfile");
		return 1;
	}
	outfile << "//Mac Address" << std::endl;
	writeMacAddr();
	writePad(6, 2);
	outfile << "//IP Address" << std::endl;
	writeIpAddr();
	outfile << "//Server Address" << std::endl;
	writeServerIpAddr();
	outfile << "//Boards" << std::endl;
	writeBoards();
	writePad(23, 17);
	outfile << "//Positions" << std::endl;
	writePositions();
	outfile << "//Quad Turnouts" << std::endl;
	writeQuadTurnouts();
	outfile << "//Quad Sensors" << std::endl;
	writeQuadSensors();
	outfile << "//Dual Turnouts" << std::endl;
	writeDualTurnouts();
	outfile << "//Sensors" << std::endl;
	writeSensors();
	outfile << "//Signal Inputs" << std::endl;
	writeSignalInputs();
	outfile << "//Signal Logic" << std::endl;
	writeSignalLogic();
	writePad(896, 32);
	outfile << "//Indicators" << std::endl;
	writeIndicators();
	writePad(944, 16);
	outfile << "//Power" << std::endl;
	writePower();
	writePad(976, 48);
	outfile << "//KBTrack" << std::endl;
	writeKBTrack();
	writePad(1280, 20);
	outfile << "//Signals" << std::endl;
	writeSignals();
	outfile << "//NOR" << std::endl;
	writeNOR();
	writePad(1604, 444);


	outfile << "//DONE" << std::endl;
	outfile.close();

	return 0;
}
