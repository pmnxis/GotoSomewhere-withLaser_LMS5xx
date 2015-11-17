#include <iostream>
#include <string>
#include <fstream>
#include "Aria.h"

void readyLog(std::ofstream &stream) {
	time_t timer;
	char buffer[120];
	std::string logPath("C:\\short_adr\\ArLog\\");
	time(&timer);
	strftime(buffer, 80, "LOG-%d-%m-%Y__%I-%M-%S.txt", localtime(&timer));
	logPath = logPath + buffer;
	std::cout << logPath << std::endl;
	stream.open(logPath.c_str());
	stream << "file name : " << logPath << std::endl;
}


int main(int argc, char **argv)
{
	Aria::init();
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobot robot;
	ArSick laser;
	std::ofstream stream; // for loggin
	time_t timer;
	char buffer[120];
	//for loggin
	ArRobotConnector robotConnector(&argParser, &robot);
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);
	int32_t count = 0;
	readyLog(stream);
	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (argParser.checkHelpAndWarnUnparsed())
		{
			// -help not given, just exit.
			Aria::logOptions();
			Aria::exit(1);
			return 1;
		}
	}


	// Trigger argument parsing
	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}

	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);

	puts("This program will make the robot wander around. It uses some avoidance\n"
		"actions if obstacles are detected, otherwise it just has a\n"
		"constant forward velocity.\n\nPress CTRL-C or Escape to exit.");

	//ArSonarDevice sonar;
	//robot.addRangeDevice(&sonar);
	robot.addRangeDevice(&laser);

	robot.runAsync(true);



	// try to connect to laser. if fail, warn but continue, using sonar only
	if (!laserConnector.connectLasers())
	{
		ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only.");
	}

	double sampleAngle, dist;
	auto sampleRange = laser.currentReadingPolar(-20, 20, &sampleAngle);
	auto degreeStr = laser.getDegreesChoicesString();

	std::cout << "auto sampleRange = laser.currentReadingPolar(-20, 20, &sampleAngle);" << sampleRange << std::endl;
	std::cout << "auto degreeStr = laser.getDegreesChoicesString(); : " << degreeStr << std::endl;


	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();
	//robot.getLaserMap()
	robot.comInt(ArCommands::SOUNDTOG, 0);

	// add a set of actions that combine together to effect the wander behavior
	ArActionStallRecover recover;
	ArActionBumpers bumpers;
	ArActionAvoidFront avoidFrontNear("Avoid Front Near", 100, 0);
	ArActionAvoidFront avoidFrontFar;
	ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
	
	robot.addAction(&recover, 100);
	robot.addAction(&bumpers, 75);
	robot.addAction(&avoidFrontNear, 50);
	robot.addAction(&avoidFrontFar, 49);
	robot.addAction(&constantVelocity, 25);

	stream << "IMPORTANT !! == robot.getRobotRadius() : " << robot.getRobotRadius << std::endl;
	std::string timestamp;
	while (1)
	{
		//

		//
		time(&timer);
		strftime(buffer, 80, " - timestamp : %I:%M:%S", localtime(&timer));
		timestamp = buffer;
		dist = robot.checkRangeDevicesCurrentPolar(-70, 70, &sampleAngle) - robot.getRobotRadius();
		stream << count << timestamp << "checkRangeDevicesCurrentPolar(-70, 70, &angle)  :  " << dist << std::endl;
		Sleep(500);
		count++;
	}

	// wwill add processor here

	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();

	Aria::exit(0);
	return 0;
}