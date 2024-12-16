#pragma once

#include <string>

#include "Turnout.h"
#include "QuadSensor.h"
#include "SignalInput.h"
#include "SignalLogic.h"
#include "Indicator.h"
#include "KBTrack.h"
#include "Signal.h"

extern std::string getSignalAspect(int);

class CMRSconfig
{
public:
	CMRSconfig();
	~CMRSconfig();

	void parse_data(const std::string&);

	void printMacAddr();
	void printIpAddr();
	void printServerIpAddr();
	void printBoards();
	void printQuadTurnouts();
	void printQuadSensors();
	void printDualTurnouts();
	void printSensors();
	void printSignalInputs();
	void printSignalLogic();
	void printIndicators();
	void printKBTrack();
	void printSignals();

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

	int norData[64];

	int byteDone = 0;



};

