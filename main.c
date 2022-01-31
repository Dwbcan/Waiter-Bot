#include "EV3Servo-lib-UW.c"
#include "PC_FileIO.c"

const tSensors PORT_GYRO = S4;
const tSensors PORT_TOUCH = S1;
const tSensors PORT_ULTRASONIC = S3;
const tSensors PORT_TETRIX = S2;

const int ANGLE_POWER = 10;
const int DRIVING_POWER = 15;

const int MAX_INSTRUCTIONS = 30;
const int MAX_ORDERS = 30;

const int TYPE_DRIVE = 0;
const int TYPE_TURN = 1;
const int TYPE_FILLORDERS = 2;

const int FULL_TURN = 360;
const int HALF_TURN = 180;
const int QUARTER_TURN = 90;

const float WHEEL_RADIUS = 4.105;
const float ENCODER_TO_CM = (PI / 180) * WHEEL_RADIUS;

const int WAIT_SECOND = 1000;
const int WAIT_FOR_CONFIRM = 3;
const int WAIT_FOR_WATER = 5;

const int TETRIX_OPEN = 100;
const int WATER_ROTATE = 120;


typedef struct //an Instruction tells the robot when and how much to move/turn, and when to fill orders
{
	int instructionType, moveValue;
} Instruction;

typedef struct //an Order specifies at which path position the order was made, and what the person making the order wants
{
	int pathPosition;
	bool isWater;
} Order;

void configure() // configures the ultrasonic, touch and gyro sensors
{
	SensorType[PORT_ULTRASONIC] = sensorEV3_Ultrasonic;
	SensorType[PORT_TOUCH] = sensorEV3_Touch;
	SensorType[PORT_GYRO] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[PORT_GYRO] = modeEV3Gyro_Calibration;
	wait1Msec(50);
	SensorMode[PORT_GYRO] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
}

// Function that reads in a file and sorts them into Instruction array and the points xSnack and ySnack
int readFromFile(TFileHandle & fin, Instruction* instructions, int & xSnack, int & ySnack)
{
	readIntPC(fin, xSnack);
	readIntPC(fin, ySnack);

	string instructionType = "";
	int instructionValue = 0;
	int index = 0;

	while(readTextPC(fin, instructionType) && readIntPC(fin, instructionValue) && index < MAX_INSTRUCTIONS)
	{
		if (instructionType == "X")
			instructions[index].instructionType = TYPE_FILLORDERS;
		else
		{
			instructions[index].moveValue = instructionValue;
			if (instructionType == "Move")
				instructions[index].instructionType = TYPE_DRIVE;
			else
				instructions[index].instructionType = TYPE_TURN;
		}
		index++;
	}

	return index;
}

void drive(int distanceCM) //Drives forward distanceCM centimetres, adjusting for unintended rotation
{
	nMotorEncoder[motorA] = 0;
	wait1Msec(WAIT_SECOND);
	resetGyro(PORT_GYRO);
	motor[motorA] = motor[motorD] = DRIVING_POWER;
	while (nMotorEncoder[motorA] * ENCODER_TO_CM < distanceCM)
	{
		if (getGyroDegrees(PORT_GYRO) > 0)
		{
			motor[motorD] = DRIVING_POWER - 2;
			motor[motorA] = DRIVING_POWER + 2;
	  }
		else if (getGyroDegrees(PORT_GYRO) < 0)
		{
			motor[motorA] = DRIVING_POWER - 4;
			motor[motorD] = DRIVING_POWER + 4;
		}
	}
	motor[motorA] = motor[motorD] = 0;
}

void rotateRobot(int angle) // Rotates by the desired angle, then adjusts for overshoot (if angle > 0, a clockwise turn is made; otherwise, a counterclockwise turn)
{
	resetGyro(PORT_GYRO);
	if(angle > 0)
	{
		motor[motorA] = DRIVING_POWER;
		motor[motorD] = -DRIVING_POWER;
	}
	else
	{
		motor[motorA] = -DRIVING_POWER;
		motor[motorD] = DRIVING_POWER;
	}
	while(abs(getGyroDegrees(PORT_GYRO)) < abs(angle))
	{}

	if(angle > 0)
	{
		motor[motorA] = -DRIVING_POWER / 5;
		motor[motorD] = DRIVING_POWER / 5;
	}
	else
	{
		motor[motorA] = DRIVING_POWER / 5;
		motor[motorD] = -DRIVING_POWER / 5;
	}
	while(abs(getGyroDegrees(PORT_GYRO)) > abs(angle))
	{}
	motor[motorA] = motor[motorD] = 0;
}

// Boolean function that detects if a person has approached the robot and wants assistance.
bool touchActivated() {
	clearTimer(T1);
	while (time1[T1] < WAIT_SECOND * WAIT_FOR_CONFIRM) {
		if (SensorValue[PORT_TOUCH])
			return true;
	}
	return false;
}

void prepArm() // Manipulates an additional motor and the Tetrix claw to grab the snack basket
{
	nMotorEncoder[motorB] = 0;
	motor[motorB] = -ANGLE_POWER;
	wait1Msec(WAIT_SECOND/2);
	nMotorEncoder[motorB] = 0;
	motor[motorB] = 0;

	wait1Msec(WAIT_SECOND);
	setServoPosition(PORT_TETRIX, 1, TETRIX_OPEN);

	motor[motorB] = ANGLE_POWER;
	wait1Msec(WAIT_SECOND/2);
	motor[motorB] = 0;
	wait1Msec(WAIT_SECOND);
	setServoPosition(PORT_TETRIX, 1, 0);
}

// Function that takes in the number of waters ordered and rotates the robot into the correct orientation(s) and waits for a person to fill up the cups secured on the robot.
void fillWater(int numWater)
{
	for (int index = 0; index < numWater; index++)
	{
		clearTimer(T1);
		while(time1[T1] < WAIT_SECOND * WAIT_FOR_WATER)
		{}
		if (index != numWater - 1)
			rotateRobot(WATER_ROTATE);
	}
	int turnAngle = (numWater - 1) * WATER_ROTATE;

	rotateRobot(-(turnAngle % FULL_TURN));
} //robot ends in same orientation it started with

bool confirmPerson()
{
	return SensorValue[PORT_ULTRASONIC] < 20;
}

void giveOrder(bool isWater) //Stop and let the person take their ordered item
{
	if (isWater)
		displayString(4, "Please take your water.");
	else
		displayString(4, "Please take your snack.");

	displayString(6, "After, press ENTER.");

	while(!getButtonPress(buttonEnter))
	{}
	while (!getButtonPress(buttonEnter))
	{}

	eraseDisplay();
	wait1Msec(WAIT_SECOND);
}

void fillOrders(Instruction* instructions, Order* orders, int & waterCount, int & snackCount, int xSnack, int ySnack, int instructionIndex, bool & hasBasket) //Retrieve all of the outstanding orders that have been made, and deliver the items back to those who ordered
{
	if (waterCount + snackCount > 0)
	{
		rotateRobot(HALF_TURN);
		for (int index = instructionIndex; index >= 0; index--)
			if (instructions[index].instructionType == TYPE_DRIVE)
			drive(instructions[index].moveValue);
		else if (instructions[index].instructionType == TYPE_TURN)
			rotateRobot(-QUARTER_TURN * instructions[index].moveValue);

		if (waterCount > 0)
			fillWater(waterCount);

		if (snackCount > 0)
		{
			rotateRobot(QUARTER_TURN * xSnack / abs(xSnack));
			drive(abs(xSnack));
			rotateRobot(-QUARTER_TURN * (xSnack * ySnack / abs(xSnack * ySnack)));
			drive(abs(ySnack));

			if (ySnack < 0)
				rotateRobot(HALF_TURN);

			if (hasBasket)
			{
				displayTextLine(7, "Please fill basket with snacks.");
				displayTextLine(8, "Press ENTER when finished.");
				while (!getButtonPress(buttonEnter))
				{}
				while (getButtonPress(buttonEnter))
				{}
				eraseDisplay();
			}
			else
			{
				prepArm();
				hasBasket = true;
			}

			if (ySnack > 0)
				rotateRobot(HALF_TURN);

			drive(abs(ySnack));

			rotateRobot(QUARTER_TURN * (xSnack * ySnack / abs(xSnack * ySnack)));
			drive(abs(xSnack));

			rotateRobot(-QUARTER_TURN * xSnack / abs(xSnack));
		}
		else
		{
			rotateRobot(HALF_TURN);
		}

		int totalPathPosition = 0;
		int orderIndex = 0;
		for (int index = 0; index <= instructionIndex; index++)
			if (instructions[index].instructionType == TYPE_DRIVE)
			{
				nMotorEncoder[motorA] = 0;
				motor[motorA] = motor[motorD] = DRIVING_POWER;
				while (nMotorEncoder[motorA] * ENCODER_TO_CM < instructions[index].moveValue)
				{
					if (orders[orderIndex].pathPosition == totalPathPosition + round(nMotorEncoder[motorA] * ENCODER_TO_CM)) {
						motor[motorA] = motor[motorD] = 0;
						giveOrder(orders[orderIndex].isWater);
						motor[motorA] = motor[motorD] = DRIVING_POWER;
						orderIndex++;
					}
				}
				motor[motorA] = motor[motorD] = 0;
				totalPathPosition += instructions[index].moveValue;
			}
			else if (instructions[index].instructionType == TYPE_TURN)
				rotateRobot(QUARTER_TURN * instructions[index].moveValue);
		waterCount = 0;
		snackCount = 0;
		for (int index = 0; index < MAX_ORDERS; index++) {
			orders[index].pathPosition = 0;
			orders[index].isWater = false;
		}
	}
}

void outputSnackWater(TFileHandle & fout, int numSnack, int numWater) //Outputs the total number of snacks and water given to a file
{
	writeLongPC(fout, numSnack);
	writeTextPC(fout, " snacks were given\n");
	writeLongPC(fout, numWater);
	writeTextPC(fout, " waters were given");
}

task main()
{
	configure();
	TFileHandle fin;
	if (!openReadPC(fin, "room_data.txt"))
	{
		displayTextLine(6, "File could no be opened.");
		displayTextLine(8, "Press any button to exit.");
		while (!getButtonPress(buttonAny))
		{}
		while (getButtonPress(buttonAny))
		{}
	}
	else
	{
		int xSnack = 0, ySnack = 0;
		Instruction instructions[MAX_INSTRUCTIONS];
		Order orders[MAX_ORDERS];

		int numInstructions = readFromFile(fin, instructions, xSnack, ySnack);

		int totalSnackCount = 0, totalWaterCount = 0;
		bool exit = false;

		do {
			int snackCount = 0, waterCount = 0;
			int totalPathPosition = 0;
			bool hasBasket = false;

			for (int index = 0; index < numInstructions; index++)
			{
				if (instructions[index].instructionType == TYPE_DRIVE)
				{
					nMotorEncoder[motorA] = 0;
					motor[motorA] = motor[motorD] = DRIVING_POWER;
					while (nMotorEncoder[motorA] * ENCODER_TO_CM < instructions[index].moveValue)
					{
						//confirm person code
						if(confirmPerson())
						{
							if (snackCount + waterCount >= MAX_ORDERS)
							{
								displayTextLine(6, "Sorry, too many orders.");
								wait1Msec(WAIT_SECOND * WAIT_FOR_CONFIRM);
								eraseDisplay();
							}
							else
							{
								motor[motorA] = motor[motorD] = 0;
								displayString(6, "Do you need assistance?");
								displayString(8, "Activate the touch sensor if yes.");
								if(touchActivated())
								{
									eraseDisplay();
									displayString(5, "Do you want a snack or water?");
									displayString(7, "Left button for snack");
									displayString(8, "Right button for water");
									while(!getButtonPress(buttonLeft) && !getButtonPress(buttonRight))
									{}
									eraseDisplay();
									if (getButtonPress(buttonRight))
										waterCount++;
									else
										snackCount++;

									orders[snackCount + waterCount - 1].pathPosition = totalPathPosition + nMotorEncoder[motorA] * ENCODER_TO_CM;
									orders[snackCount + waterCount - 1].isWater = getButtonPress(buttonRight);
									while (getButtonPress(buttonAny))
									{}
									wait1Msec(WAIT_SECOND);
                }
                else
                  eraseDisplay();
							}
							motor[motorA] = motor[motorD] = DRIVING_POWER;
						}
					}
					motor[motorA] = motor[motorD] = 0;
					totalPathPosition += instructions[index].moveValue;
				}
				else if (instructions[index].instructionType == TYPE_TURN)
					rotateRobot(QUARTER_TURN * instructions[index].moveValue);
				else
				{
					totalSnackCount += snackCount;
					totalWaterCount += waterCount;
					fillOrders(instructions, orders, waterCount, snackCount, xSnack, ySnack, index, hasBasket);
				}
				nMotorEncoder[motorA] = 0;
				wait1Msec(WAIT_SECOND);
			}

			rotateRobot(HALF_TURN);
			for (int index = numInstructions - 1; index >= 0; index--)
				if (instructions[index].instructionType == TYPE_DRIVE)
				drive(instructions[index].moveValue);
			else if (instructions[index].instructionType == TYPE_TURN)
				rotateRobot(-QUARTER_TURN * instructions[index].moveValue);

			rotateRobot(HALF_TURN);
			displayString(5, "Would you like to repeat");
			displayString(6, "the path?");
			displayString(8, "Left button for yes");
			displayString(9, "Right button for no");
			while (!getButtonPress(buttonLeft) && !getButtonPress(buttonRight))
			{}
			if (getButtonPress(buttonRight))
				exit = true;
		} while (!exit);

		TFileHandle fout;
		if (openWritePC(fout, "totals.txt"))
			outputSnackWater(fout, totalSnackCount, totalWaterCount);
	}
}