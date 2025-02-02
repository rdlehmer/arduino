#pragma once

#include <string>
#include <fstream>

#include "Turnout.h"
#include "QuadSensor.h"
#include "SignalInput.h"
#include "SignalLogic.h"
#include "Indicator.h"
#include "KBTrack.h"
#include "Signal.h"

#define CMRS_STRING_SIZE    7
#define CMRS_MACADDR_OFFSET 0
#define CMRS_MACADDR_SIZE   6
#define CMRS_IPADDR_OFFSET  8
#define CMRS_IPADDR_SIZE	4
#define CMRS_SIPADDR_OFFSET	12
#define CMRS_SIPADDR_SIZE	4
#define CMRS_BOARDS_OFFSET	16
#define CMRS_BOARDS_SIZE	7
#define CMRS_QUADPOS_OFFSET 40
#define CMRS_QUADPOS_SIZE	16
#define CMRS_DUALPOS_OFFSET	56
#define CMRS_DUALPOS_SIZE	8
#define CMRS_QUADTURNOUT_OFFSET	64
#define CMRS_TURNOUT_SIZE	8
#define CMRS_QUADTURNOUT_NUMBER	16
#define CMRS_QUADSENSOR_OFFSET 192
#define CMRS_QUADSENSOR_SIZE	10
#define CMRS_QUADSENSOR_NUMBER 16
#define CMRS_DUALTURNOUT_OFFSET 352
#define CMRS_DUALTURNOUT_NUMBER 8
#define CMRS_SENSOR_OFFSET      416
#define CMRS_SENSOR_SIZE		8
#define CMRS_SENSOR_NUMBER		16
#define CMRS_SIGNALINPUT_OFFSET 544
#define CMRS_SIGNALINPUT_SIZE	9
#define CMRS_SIGNALINPUT_NUMBER 32
#define CMRS_SIGNALLOGIC_OFFSET 832
#define CMRS_SIGNALLOGIC_SIZE   4
#define CMRS_SIGNALLOGIC_NUMBER 16
#define CMRS_INDICATORS_OFFSET  928
#define CMRS_INDICATORS_SIZE	2
#define CMRS_INDICATORS_NUMBER  8
#define CMRS_POWER_OFFSET		960
#define CMRS_KBTRACK_OFFSET		1024
#define CMRS_SIGNAL_OFFSET		1300
#define CMRS_SIGNAL_SIZE		15
#define CMRS_SIGNAL_NUMBER		16
#define CMRS_NOR_OFFSET			1540
#define CMRS_NOR_SIZE			64


extern std::string getSignalAspect(int);

class CMRSconfig
{
public:
	CMRSconfig();
	~CMRSconfig();

	void parse_data(const std::string&);
	int writeData();

	void writePad(int, int);

	void printMacAddr();
	void writeMacAddr();

	void printIpAddr();
	void writeIpAddr();

	void printServerIpAddr();
	void writeServerIpAddr();

	void printBoards();
	void writeBoards();

	void writePositions();

	void printQuadTurnouts();
	void writeQuadTurnouts();

	void printQuadSensors();
	void writeQuadSensors();

	void printDualTurnouts();
	void writeDualTurnouts();

	void printSensors();
	void writeSensors();

	void printSignalInputs();
	void writeSignalInputs();

	void printSignalLogic();
	void writeSignalLogic();

	void printIndicators();
	void writeIndicators();

	void writePower();

	void printKBTrack();
	void writeKBTrack();

	void printSignals();
	void writeSignals();

	void writeNOR();

private:
	int macaddr[6];
	int ipaddr[4];
	int serveripaddr[4];

	int boards[7];
	int quadpos[16];
	int dualpos[8];
	Turnout quadTurnout[16];
	QuadSensor quadSensor[16];
	Turnout dualTurnout[8];
	Turnout sensor[16];
	SignalInput signalInput[32];
	SignalLogic signalLogic[16];
	Indicator indicator[8];

	int power[16];
	KBTrack kbTrack[16];

	Signal signal[16];

//	int norData[64];

	int byteDone = 0;

	std::ofstream outfile;

};

