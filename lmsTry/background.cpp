#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#include <Aria.h>

class autoWorker{
private:
	char cstrbuff[500];

	ArRobot * linkedArRobot;
	ArLaser * linkedArLaser;
	ArSick * linkedArSick;
protected:
	std::string logDir = "C:\\short_adr\\ArLog\\";
	bool logReady;
	std::ofstream logStream;
	time_t timer;
	void createLogStream(const char filename[]) {
		time(&(this->timer));
		strftime(cstrbuff, 48, "LOG-%Y-%m-%d-time-%I-%M-%S.txt", localtime(&(this->timer)));
		std::string temp;
		temp = logDir + cstrbuff;
		logStream.open(temp.c_str());
		logStream << "ROBOT LOG : " << this->linkedArRobot->getName() << "\nStart Time : " << cstrbuff << std::endl;
		return;
	}

	void InitLaser() {
		this->linkedArRobot->addRangeDevice((this->linkedArSick));



	}

// for pulbic
public:

	autoWorker(ArRobot * robot) {
		this->linkedArRobot = robot;
		std::cout << "new autoWorker Added , name : " << linkedArRobot->getName() << "." << std::endl;
	}

	autoWorker(ArRobot * robot, ArSick * sick ,ArLaser * laser) {
		linkedArRobot = robot;
		std::cout << "new autoWorker Added , name : " << linkedArRobot->getName() << "." << std::endl;
		this->linkedArLaser = laser;
		this->linkedArSick = sick;
	}

	double duplicated_currentReadingPolar(double startAngle,
		double endAngle,
		double *angle) const
	{
		ArPose pose;
		if (this->linkedArRobot != NULL)
			pose = this->linkedArRobot->getPose();
		else
		{
			ArLog::log(ArLog::Normal, "ArRangeDevice %s: NULL robot, won't get polar reading correctly", getName());
			pose.setPose(0, 0);
		}
		


		/*
		return myCurrentBuffer.getClosestPolar(startAngle, endAngle,
			pose,
			myMaxRange,
			angle);
			*/
	}



	void autoLogger() {
		



	}

	void actProcesser(void * func) {
		//asdfasdfasdf
	}

};