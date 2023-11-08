#define _USE_MATH_DEFINES
#include <Windows.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <sstream>
#include <826api.h>
using namespace std;

int SetDacOutput(uint, uint, uint, double);

int SetDacOutput(uint board, uint chan, uint range, double volts)
{
	uint setpoint;

	// conversion is based on dac output range:
	switch (range)
	{
	case S826_DAC_SPAN_0_5:  // 0 to +5V
		setpoint = (uint)(volts * 0xFFFF / 5);
		break;
	case S826_DAC_SPAN_0_10:  // 0 to +10V
		setpoint = (uint)(volts * 0xFFFF / 10);
		break;
	case S826_DAC_SPAN_5_5:  // -5V to +5V
		setpoint = (uint)(volts * 0xFFFF / 10) + 0x8000;
		break;
	case S826_DAC_SPAN_10_10: // -10V to +10V
		setpoint = (uint)(volts * 0xFFFF / 20) + 0x8000;
		break;
	}

	return S826_DacDataWrite(board, chan, setpoint, 0);  // program DAC output
}

double AngleCalc(unsigned int value, float GearRatio, int check, bool base)
{
	const unsigned int maxData = 0xFFFFFFFF;
	double outputValue = 0;

	//from simulove for alt angle test
	double K1 = 0.000128416;
	double K2 = 0.000140087;

	if (check == 1)
	{
		value -= 100000;
	}
	if (value >= (maxData / 2))
	{
		outputValue = (double)value - (double)maxData;
	}
	else
	{
		outputValue = value;
	}

	//// angle calc from Guy's chai code
	//outputValue *= 2 * M_PI;
	//outputValue /= 4000;
	//outputValue /= GearRatio;

	if (base == true) {
		//calc from simusolve
		outputValue = outputValue * K1;
	}
	else {
		//calc from simusolve
		outputValue = outputValue * K2;
	}


	return outputValue;

}

double PositionCalc(vector<unsigned int> ENCDAT, int check, ofstream& posno, ofstream& angno, vector<double>& data)
{
	float gearRatio[3];
	double Position[3];
	double x, y, z;
	double angles[3];
	double sin[4];
	double cos[4];

	const float length1 = 0.203f; // extra link lentgh (along z axis)
	const float length2 = 0.215f; // parallel linkage length
	const float length3 = 0.203f; // end effector linkage
	const float phantomHeightDiff = 0.087f;
	//    const float phantomYDisplacement = 0.035f; //added to lengths 1 & 2 so now obsolete (is thimble length)

	string msg, msg2;
	stringstream strg, strg2;

	//bools to crontol scalar values used for simulsolve method
	bool base = true;
	bool arm = false;

	gearRatio[0] = 13.4f;
	gearRatio[1] = 11.5f;
	gearRatio[2] = 11.5f;

	//modified angle calc from Guy's chai code
	angles[0] = AngleCalc(ENCDAT[2], gearRatio[0], check, base);// Calculate angles from encoder values
	angles[1] = AngleCalc(ENCDAT[0], gearRatio[1], check, arm); // Calculate angles from encoder values
	angles[2] = AngleCalc(ENCDAT[1], gearRatio[2], check, arm); // Calculate angles from encoder values

	for (unsigned int channel = 0; channel < 3; channel++)
	{

		if (channel != 0)//this prevents the seperator being placed at the begining or end of the string
		{
			strg2 << " ";
		}
		strg2 << angles[channel];
	}
	msg2 = strg2.str();
	strg2.str(""); //code to clear the stringstream

	angno << msg2; //attempt with type string instead of int

	// Pre Calculate Trig
	sin[1] = std::sin(angles[0]);
	sin[2] = std::sin(angles[1]);
	sin[3] = std::sin(angles[2]);
	cos[1] = std::cos(angles[0]);
	cos[2] = std::cos(angles[1]);
	cos[3] = std::cos(angles[2]);

	// Calculate Position
	x = (cos[1] * ((length2 * cos[2]) + (length3 * sin[3])) - length2);
	y = sin[1] * ((length2 * cos[2]) + (length3 * sin[3]));
	z = length1 - (length3 * cos[3]) + (length2 * sin[2]);

	//rotate co-ord frame to desired

	Position[0] = x;
	Position[1] = y;
	Position[2] = z;


	for (size_t i = 0; i < 3; ++i)
	{
		if (i != 0)//this prevents the seperator being placed at the begining or end of the string
		{
			strg << " ";
		}
		strg << Position[i];
	}
	msg = strg.str();
	strg.str(""); //code to clear the stringstream

	posno << msg; //attempt with type string instead of int

	x = 0.0;
	y = 0.0;
	z = 0.0;
	return 0;
}
double getPosition(vector<unsigned int> ENCDAT, int check, vector<double>& data, double& x, double& y, double& z)
{
	float gearRatio[3];
	double angles[3];
	double sin[4];
	double cos[4];

	const float length1 = 0.203f; // extra link lentgh (along z axis)
	const float length2 = 0.215f; // parallel linkage length
	const float length3 = 0.203f; // end effector linkage
	const float phantomHeightDiff = 0.087f;
	//    const float phantomYDisplacement = 0.035f; //added to lengths 1 & 2 so now obsolete (is thimble length)

	string msg, msg2;
	stringstream strg, strg2;

	//bools to crontol scalar values used for simulsolve method
	bool base = true;
	bool arm = false;

	gearRatio[0] = 13.4f;
	gearRatio[1] = 11.5f;
	gearRatio[2] = 11.5f;

	//modified angle calc from Guy's chai code
	angles[0] = AngleCalc(ENCDAT[2], gearRatio[0], check, base);// Calculate angles from encoder values
	angles[1] = AngleCalc(ENCDAT[0], gearRatio[1], check, arm); // Calculate angles from encoder values
	angles[2] = AngleCalc(ENCDAT[1], gearRatio[2], check, arm); // Calculate angles from encoder values

	
	// Pre Calculate Trig
	sin[1] = std::sin(angles[0]);
	sin[2] = std::sin(angles[1]);
	sin[3] = std::sin(angles[2]);
	cos[1] = std::cos(angles[0]);
	cos[2] = std::cos(angles[1]);
	cos[3] = std::cos(angles[2]);

	// Calculate Position
	x = (cos[1] * ((length2 * cos[2]) + (length3 * sin[3])) - length2);
	y = sin[1] * ((length2 * cos[2]) + (length3 * sin[3]));
	z = length1 - (length3 * cos[3]) + (length2 * sin[2]);


	return 0;
}


int main()
{
	// Open connection
	int id, flags = S826_SystemOpen();
	int check;
	vector<unsigned int> ENCDAT1; //variable for storing the encoder data
	vector<unsigned int> ENCDAT2; //variable for storing the encoder data
	vector<double> data1;
	vector<double> data2;
	unsigned int temp[3] = { 0, 0, 0 };
	unsigned int ctstamp = 0;
	unsigned int reason = 0;
	int tCalibration = 0;
	int tPosition = 0;
	string msg;
	stringstream strg;
	uint boardNum;
	ofstream encno;
	ofstream posno;
	ofstream angno;
	double x1, y1, z1, x2, y2, z2;


	if (flags < 0)
		cout << "S826_SystemOpen returned error code " << flags << endl;
	else if (flags == 0)
		cout << "No boards were detected" << endl;
	else {
		cout << "Boards were detected with these IDs: ";
		for (id = 0; id < 16; id++) {
			if (flags & (1 << id))
			{
				cout << id << " " << endl;
				boardNum = id;
			}
		}
	}

	cout << "For zero start enter 0 " << endl;
	cin >> check;
	cout << '\n';
	cout << "Using board: " << boardNum << endl;
	cout << "Hold the device in its initial configuration and press ENTER key to continue." << endl;
	cin.ignore();

	if (check == 0) {
		// Configure encoder interface
		for (uint ch = 0; ch < 6; ++ch)
		{
			S826_CounterStateWrite(boardNum, ch, 0); // Stop channel operation
			S826_CounterModeWrite(boardNum, ch, S826_CM_K_QUADX4); // Configure counter
			S826_CounterStateWrite(boardNum, ch, 1); // Start channel operation
			S826_CounterPreloadWrite(boardNum, ch, 0, 0); // Set preload register
			S826_CounterPreload(boardNum, ch, 1, 0); // Copy preload value to counter
		}
		encno.open("encodernumbers0.txt");
		posno.open("positionnumbers0.txt");
		angno.open("angles0.txt");
	}
	else {
		// Configure encoder interface
		for (uint ch = 0; ch < 6; ++ch)
		{
			S826_CounterStateWrite(boardNum, ch, 0); // Stop channel operation
			S826_CounterModeWrite(boardNum, ch, S826_CM_K_QUADX4); // Configure counter
			S826_CounterStateWrite(boardNum, ch, 1); // Start channel operation
			S826_CounterPreloadWrite(boardNum, ch, 0, 100000); // Set preload register
			S826_CounterPreload(boardNum, ch, 1, 0); // Copy preload value to counter
		}
		encno.open("encodernumbers1.txt");
		posno.open("positionnumbers1.txt");
		angno.open("angles0.txt");
	}


	// Configure analog output
	for (uint ch = 0; ch < 7; ++ch)
	{
		S826_DacRangeWrite(boardNum, ch, S826_DAC_SPAN_10_10, 0);
		//SetDacOutput(boardNum, ch, S826_DAC_SPAN_5_5, 0);
	}

	// Set output to 0V
	for (uint ch = 0; ch < 7; ++ch)
	{
		SetDacOutput(boardNum, ch, S826_DAC_SPAN_10_10, 0);
		//SetDacOutput(boardNum, ch, S826_DAC_SPAN_5_5, 0);
	}

	// System Calibration
	while (tCalibration < 5000)
	{

		ENCDAT1.clear();
		ENCDAT2.clear();
		//code to read the encoder data and add into the vector
		for (unsigned int channel = 3; channel < 6; channel++)
		{
			S826_CounterSnapshot(boardNum, channel); // Create Snapshot
			S826_CounterSnapshotRead(boardNum, channel - 3, &temp[channel - 3], &ctstamp, &reason, 0); // Read from snapshot
			ENCDAT1.push_back(temp[channel - 3]);
			S826_CounterSnapshotRead(boardNum, channel, &temp[channel - 3], &ctstamp, &reason, 0); // Read from snapshot
			ENCDAT2.push_back(temp[channel - 3]);
		}
		PositionCalc(ENCDAT1, check, posno, angno, data1);
		angno << "     ";
		posno << "     ";
		PositionCalc(ENCDAT2, check, posno, angno, data2);
		angno << '\n';
		posno << '\n';

		//code to convert the vector into a string (const char array)
		//code for converting the unsigned int vector into a string
		for (size_t i = 0; i < ENCDAT1.size(); ++i)
		{
			if (i != 0)//this prevents the seperator being placed at the begining or end of the string
			{
				strg << " ";
			}
			strg << ENCDAT1[i];
		}
		msg = strg.str();
		strg.str(""); //code to clear the stringstream

		encno << msg; //attempt with type string instead of int
		encno << "     ";
		for (size_t i = 0; i < ENCDAT2.size(); ++i)
		{
			if (i != 0)//this prevents the seperator being placed at the begining or end of the string
			{
				strg << " ";
			}
			strg << ENCDAT2[i];
		}
		msg = strg.str();
		strg.str(""); //code to clear the stringstream

		encno << msg; //attempt with type string instead of int

		encno << '\n';
		tCalibration++;
	}
	encno.close();
	posno.close();
	angno.close();

	// Get System Position
	while ( tPosition < 500000) {
		ENCDAT1.clear();
		ENCDAT2.clear();
		for (unsigned int channel = 3; channel < 6; channel++)
		{
			S826_CounterSnapshot(boardNum, channel); // Create Snapshot
			S826_CounterSnapshotRead(boardNum, channel - 3, &temp[channel - 3], &ctstamp, &reason, 0); // Read from snapshot
			ENCDAT1.push_back(temp[channel - 3]);
			S826_CounterSnapshotRead(boardNum, channel, &temp[channel - 3], &ctstamp, &reason, 0); // Read from snapshot
			ENCDAT2.push_back(temp[channel - 3]);
		}
		getPosition(ENCDAT1, check, data1, x1, y1, z1);
		getPosition(ENCDAT2, check, data2, x2, y2, z2);
		std::cout << "x1: " << x1 << "y1: " << y1 << "z1: " << z1 << std::endl;
		tPosition++;
	}
	// Close connection
	S826_SystemClose();

	cout << "Calibration is completed" << endl;
	cout << "Press ENTER key to continue...";
	cin.ignore();

	return 0;
}