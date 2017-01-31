#include <iostream>
#include <memory>
#include <string>


#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <math.h>
#include "CANTalon.h"
#include "AHRS.h"
#include <Joystick.h>
#include <Solenoid.h>
#include <DoubleSolenoid.h>

class Robot: public frc::IterativeRobot {
	CANTalon LF_motor;
	CANTalon LR_motor;
	CANTalon RF_motor;
	CANTalon RR_motor;
	CANTalon shooter_motor;
	Joystick controller1;
	AHRS *ahrs;
	DoubleSolenoid armControl;
	Joystick controller2;

public:
	Robot() :
		LF_motor(1),
		LR_motor(3),
		RF_motor(2),
		RR_motor(4),
		shooter_motor(2),
		controller1(0),
		armControl(0, 1),
		controller2(1)

{
		ahrs = new AHRS(SPI::Port::kMXP);
}
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		lw = LiveWindow::GetInstance();
		LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
	}

	int stopturn=0;
	int timr=0;
	double teleMaxSpeed=0.5;

	// ------------------------------------------------------
	// Motors variables
	// ------------------------------------------------------

	// These are all wrong, probably:
	int iMotorLeftFront = 0;
	int iMotorLeftRear = 1;
	int iMotorRightFront = 2;
	int iMotorRightRear = 3;
	int iMotorLifter = 4;

	int iPneumFlaps = 0;
	int iPneumMandibles = 1;
	int iPneumWhatever = 2;

	bool armMoving=false;
	// ------------------------------------------------------
	// Controller variables
	// ------------------------------------------------------

	int iJoystickX_ = 0; // Forward motion
	int iJoystickY_ = 1; // Side motion
	int iJoystickRotate_ = 2; //
	int iJoystickFast_ = 8; //
	int iJoystickSlow_ = 7; //
	int iJoystickArm = 2;

	int iJoystick2A_ = 2;
	int iJoystick2B_ = 3;
	double shootSpeed = 0.3;
	/*
	int iJoystickOpenGearFlap_ = 0; // 
	int iJoystickOpenGearFlap_ = 0; // 
	int iJoystickOpenGearFlap_ = 0; // 

	int i_ = 0;
	int i_ = 0;
	int i_ = 0;
	int i_ = 0;
	int i_ = 0;
	*/


	// ------------------------------------------------------
	// General Functions
	// ------------------------------------------------------

	// Test function that goes through each of the functions
	// and validates that every function is working
	void TestEverything() {
		// run through all of the functions with some values (+ and -)
	}

	// Read Config File
	void ReadConfigFile() {
		// Reads in the configuration file and sets all of the variables to defaults
	}

	// Put robot status variables on the SmartDashboard
	void DisplayRobotStatus() {
		double ShooterSpeed;
		bool shootOK;
		float CurrentAngle;
		CurrentAngle = ahrs->GetYaw(); //Getting what angle we are at
		SmartDashboard::PutNumber(  "CurrentAngle", CurrentAngle);

		ShooterSpeed = shooter_motor.GetSpeed();
		SmartDashboard::PutNumber(  "Shoot Speed", ShooterSpeed);
		shootOK = (fabs(ShooterSpeed) - targetSpeed < 0.05);
		SmartDashboard::PutBoolean( "Target Speed Reached", shootOK);

	}

	// ------------------------------------------------------
	// Climbing Functions
	// ------------------------------------------------------

	// ClimbUp does:
	// Simply turn on the correct motor when a given button is pushed
	// UpDown sets the direction (up or down)

	void ClimbUp(int UpDown) {

		// We need to get a timer from some place to allow us to
		// limit when the climber motor turns on

	}

	// ------------------------------------------------------
	// Gear Functions
	// ------------------------------------------------------

	// Toggle a button and the door opens fully
	// Toggle the same button and the door closes fully
	// Need a state variable for whether the door is open or closed

	// OpenCloseGearPanels does:
	// Opens or closes the door depending on the state and the 
	// position of the button
	void OpenCloseGearPanels() {

		// single button opens or closes door:
		// push air or suck air (different functions)

	}

	// Pick gear up functionality:
	//  - lower arm
	//  - actuate claw (close)
	//  - raise arm
	//  - actuate claw (open)

	void MoveArm(bool UpOrDown) {// Up is true, and down is false

		if(UpOrDown){
			armControl.Set(DoubleSolenoid::Value::kReverse);
		}
		else{
			armControl.Set(DoubleSolenoid::Value::kForward);
		}

		// move the arm up or down via pneumatics
	}	

	void ActuateClaw(int OpenOrClose) {
		// Open or close the claw
	}

	// Do all 4 steps of picking up the gear:
	void PickUpGear() {
/*
		MoveArm(iDown_);
		ActuateClaw(iClosed_);
		MoveArm(iUp_);
		ActuateClaw(iOpen_);
*/
	}

	// This is for the key picker upper
	// Mandibles should be actuated open at the start
	// Driver should be able to actuate them though (button!)
	void MoveMandible() {

		// move mandibles open or closed
	}

	// ------------------------------------------------------
	// Sweeping Functions
	// ------------------------------------------------------

	// Just need to turn the motor on and off
	void SweeperMotorToggle() {
		// Should put the speed in the smart dash board?
		// Just toggle motor on or off
	}

	// ------------------------------------------------------
	// Shooting Functions
	// ------------------------------------------------------

	// Move blocking out of the way
	void ShootOpenHopper() {
		// Toggles hopper open or closed
	}

	// Turn on motor for shooting
	// Question: do we ho

	void ShooterToggleMotor() {
		// Toggles motor on or off
	}

	bool isADown=false;
	bool isBDown=false;
	void ShooterChangeMotorSpeed() {
		if (controller2.GetRawButton(iJoystick2A_) && !isADown){
			shootSpeed = shootSpeed+0.01;
			isADown=true;
		}
		else if(!controller2.GetRawButton(iJoystick2A_) && isADown){
			isADown=false;
		}
		if (controller2.GetRawButton(iJoystick2B_) && !isBDown){
			shootSpeed = shootSpeed-0.01;
			isBDown=true;
		}
		else if (!controller2.GetRawButton(iJoystick2B_) && isBDown){
			isBDown=false;
		}
		if (shootSpeed>1){
			shootSpeed=1;
		}
		if (shootSpeed<0){
			shootSpeed=0;
		}
		// Increase or decrease the speed
		// Sensitive, but discrete
		// This could be exactly the same as ShooterToggleMotor, but
		// changes speed variable, then sets the motor again

		// We can actually read the motor speed and increase/decrease until
		// we get to that speed
	}

	double targetSpeed = 0.3;
	double setSpeed = 0;
	double pastError = 0;
	double kP = 0.1;
	double kD = 0.05;
	void AutoSpeed() {
		double actualSpeed;
		double motorspeed;
		double errorSpeed;

		motorspeed = RF_motor.GetSpeed();
		SmartDashboard::PutNumber ("Speed test", motorspeed);
		actualSpeed = (-motorspeed-20)/500;
		errorSpeed = (targetSpeed-actualSpeed);

		setSpeed = setSpeed + errorSpeed*kP + (errorSpeed - pastError)*kD;

		if (setSpeed>1)
		{
			setSpeed = 1;
		}
		if (setSpeed<0)
		{
			setSpeed = 0;
		}
		RF_motor.Set(setSpeed);
		pastError = errorSpeed;
		SmartDashboard::PutNumber ("actual Speed", actualSpeed);
		SmartDashboard::PutNumber ("set Speed", setSpeed);
		SmartDashboard::PutNumber ("target Speed", targetSpeed);
	}

	void ShooterChangeAzimuth(int ShooterDirection) {
		// Moves the azimuth of the shooter left or right
		// depending on the ShooterDirection
	}

	//float ShooterMotorSpeed; // (Set from config file and/or Smart Dash Board)

	// ------------------------------------------------------
	// Camera Functions
	// ------------------------------------------------------

	// Not sure here!

	// ------------------------------------------------------
	// Vision Functions
	// ------------------------------------------------------

	// GearVisionReadPi:
	//   - read from network tables
	//   - Can we get the centers and sizes of the pieces of tape?
	//   - We need to then take this and turn them into a range, translation, and rotation
	//
	// ShooterVisionReadPi:
	//   - read from network tables
	//   - Can we get the centers and sizes of the pieces of tape?
	//   - We need to then take this and turn them into a range, translation, and rotation
	//

	// ------------------------------------------------------
	// Moving Functions
	// ------------------------------------------------------


	// Nominal Speed is the normal maximum speed for the robot:
	float NominalSpeed = 0.5;

    	// Nominal speed is always 50%

	// SpeedIncrease does:
	// - Left trigger causes speed to be 100%
	void SpeedIncrease();

	// SpeedDecrease does:
	// - Right trigger causes speed to be 25%
	void SpeedDecrease();

	void MecDrive(float iJoystickX_, float iJoystickY_, float iJoystickRotate_){
		double LF= -iJoystickY_ - iJoystickX_ - iJoystickRotate_;
		double RF= iJoystickY_ - iJoystickX_ - iJoystickRotate_;
		double RR= iJoystickY_ + iJoystickX_ - iJoystickRotate_;
		double LR= -iJoystickY_ + iJoystickX_ - iJoystickRotate_;

		if(fabs(LF)>1){
			LF=fabs(LF)/LF;
		}

		if(fabs(RF)>1){
			RF=fabs(RF)/RF;
		}

		if(fabs(RR)>1){
			RR=fabs(RR)/RR;
		}

		if(fabs(LR)>1){
			LR=fabs(LR)/LR;
		}

		LF_motor.Set(LF*teleMaxSpeed);
		RF_motor.Set(RF*teleMaxSpeed);
		RR_motor.Set(RR*teleMaxSpeed);
		LR_motor.Set(LR*teleMaxSpeed);
	}


	void StopDrive()
	{
		LF_motor.Set(0);
		RF_motor.Set(0);
		RR_motor.Set(0);
		LR_motor.Set(0);
	}
	void DriveForward(float DSpeed) //Defining what motors to activate to go different direction in a function
	{
		LF_motor.Set(-DSpeed);
		RF_motor.Set(DSpeed);
		RR_motor.Set(DSpeed);
		LR_motor.Set(-DSpeed);
	}
	void DriveBackward(float DSpeed) //Function to go forward
	{
		LF_motor.Set(DSpeed);
		RF_motor.Set(-DSpeed);
		RR_motor.Set(-DSpeed);
		LR_motor.Set(DSpeed);
	}

	void DriveRight(float DSpeed) //Function to drive right
	{
		LF_motor.Set(-DSpeed);
		RF_motor.Set(-DSpeed);
		RR_motor.Set(DSpeed);
		LR_motor.Set(DSpeed);
	}
	void DriveLeft(float DSpeed) //Function to drive left
		{
			LF_motor.Set(DSpeed);
			RF_motor.Set(DSpeed);
			RR_motor.Set(-DSpeed);
			LR_motor.Set(-DSpeed);
		}
	void RotateRight(float DSpeed) //Function to rotate right
		{
			LF_motor.Set(-DSpeed);
			RF_motor.Set(-DSpeed);
			RR_motor.Set(-DSpeed);
			LR_motor.Set(-DSpeed);
		}
	void RotateLeft(float DSpeed) //Function to turn left
		{
			LF_motor.Set(DSpeed);
			RF_motor.Set(DSpeed);
			RR_motor.Set(DSpeed);
			LR_motor.Set(DSpeed);
		}
	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
		ahrs->ZeroYaw();


		/*timr=0; //Making the timer = to 0
		ahrs->ZeroYaw(); //Setting current angle to zero

		SmartDashboard::PutNumber("timr1", 87);
		SmartDashboard::PutNumber("timr2", 100);
		SmartDashboard::PutNumber("timr3", 300);
		SmartDashboard::PutNumber("angle", 60);
		SmartDashboard::PutNumber("Forward 1 Speed", .2);
		SmartDashboard::PutNumber("AngleSloop", 2);
		SmartDashboard::PutNumber("MinSpeed", 0.13);
		SmartDashboard::PutNumber("timr4", 310);
		SmartDashboard::PutNumber("mastertimr", 400);
		SmartDashboard::PutNumber("Place", 1);
		SmartDashboard::PutNumber("timr21", 30);
	 	SmartDashboard::PutNumber("Forward  Speed", .3);
		SmartDashboard::PutNumber("Forward 1 Speed", .2);*/
	}

	void AutonomousPeriodic() {

		/*
		int Place=SmartDashboard::GetNumber("Place", 2);
		int timr21=SmartDashboard::GetNumber("timr21", 42);
		int timr1=SmartDashboard::GetNumber("timr1", 60); //Making different time stamps and making the available in smart dash-board
		int timr2=SmartDashboard::GetNumber("timr2", 100);
		int timr3=SmartDashboard::GetNumber("timr3", 300);
		int timr4=SmartDashboard::GetNumber("timr4", 310);
		int mastertimr=SmartDashboard::GetNumber("mastertimr", 400);
		float TurnToAngle = SmartDashboard::GetNumber("angle", 60); // The angle to turn to
		float AngleSloop = SmartDashboard::GetNumber("AngleSloop", 2);; // The amount of error we can have in degrees from the angle we need to go to
		float CurrentAngle, AngleDiff, Speed;
		float MinSpeed=SmartDashboard::GetNumber("MinSpeed", 0.13); //Minimum speed we can slow to when turning
		CurrentAngle = ahrs->GetYaw(); //Getting what angle we are at
		SmartDashboard::PutNumber(  "CurrentAngle", CurrentAngle);
		SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw()); //Gets yaw from the gyroscope
 //Putting our current angle into smart dash-board
		float ZeroAngle=0;
		float AngleDiff2;
		AngleDiff2 = CurrentAngle - ZeroAngle;
		AngleDiff = CurrentAngle - TurnToAngle; //How close are we to our angle that we want to be at
		if(Place==2){
			if(timr<timr21){
			DriveForward(SmartDashboard::GetNumber("Forward 2.1 Speed", .3));
				if (AngleDiff2 > AngleSloop) { // If we have gone over the desired angle then we turn back
						LF_motor.Set(-0.2);
						RF_motor.Set(0.3);
						RR_motor.Set(0.3);
						LR_motor.Set(-0.3);
			} if (AngleDiff2 < AngleSloop) { // If we aren't with in two degrees of our desired angle then we turn right.

						LF_motor.Set(-0.3);
						RF_motor.Set(0.2);
						RR_motor.Set(0.3);
						LR_motor.Set(-0.3);

					}//Puts how fast we go forward in smart dash-board
			}
			else{
				LF_motor.Set(0);
				RF_motor.Set(0);
				RR_motor.Set(0);
				LR_motor.Set(0);
			}
		}
		if(timr<mastertimr && Place==1){ //The master timer making sure the auton code dies after 8 seconds
			if(timr < timr1) //Only drive forward if we are less than a certain time
			{
				DriveForward(SmartDashboard::GetNumber("Forward 1 Speed", .3));
				if (AngleDiff2 > AngleSloop) { // If we have gone over the desired angle then we turn back
										LF_motor.Set(-0.2);
										RF_motor.Set(0.3);
										RR_motor.Set(0.3);
										LR_motor.Set(-0.3);
							} if (AngleDiff2 < AngleSloop) { // If we aren't with in two degrees of our desired angle then we turn right.

										LF_motor.Set(-0.3);
										RF_motor.Set(0.2);
										RR_motor.Set(0.3);
										LR_motor.Set(-0.3);
									}
			}
			if(timr>timr1 && timr<timr2){ //in between timer 1s time and timer 2s time it is dead
				LF_motor.Set(0);
				RF_motor.Set(0);
				RR_motor.Set(0);
				LR_motor.Set(0);
			}
			if(fabs(AngleDiff)<AngleSloop && stopturn==1){
				stopturn++;
			}
			if(fabs(AngleDiff)>AngleSloop && timr < timr3 && timr >timr2 && stopturn==0){ // If we are in this time frame, and we align to the angle
				Speed = fabs(AngleDiff / 180.0); //Makes robot accelerate towards the desired angle
				if (Speed < MinSpeed) Speed = MinSpeed; //Sets the minimal speed we can go when turning to the angle

				if (timr < timr3) { //Makes sure we are in a time frame

					if (AngleDiff > 0.0) { // If we have gone over the desired angle then we turn back
						RotateLeft(Speed);
					} else { // If we aren't with in two degrees of our desired angle then we turn right.
						RotateRight(Speed);
					}
				}

			}
			if(timr>timr4){
				DriveForward(SmartDashboard::GetNumber("Forward 2 Speed", .13));
			}
		}
		else if (Place==1) { // If we go over the master timers time frame then we kill the robot
			LF_motor.Set(0);
			RF_motor.Set(0);
			RR_motor.Set(0);
			LR_motor.Set(0);

		}
		timr++; //Adds time to timer
				if (fabs(CurrentAngle - TurnToAngle)>AngleSloop) {   PUT START COMMENT HERE
						RotateRight(0.2);
					} else {
						LF_motor.Set(0);
						RF_motor.Set(0);
						RR_motor.Set(0);
						LR_motor.Set(0);
					} */
/*
		if(timr<25)
		{
			RotateRight(0.3);
		}
		if(timr<50 && timr>25)
		{
			RotateLeft(0.3);
		}
		*/
		/*if (timr<100)
		{
			LF_motor.Set(-1);
			RF_motor.Set(0);
			RR_motor.Set(0);
			LR_motor.Set(0);


		}
		if (timr>100 && timr<200)
				{
					LF_motor.Set(0);
					RF_motor.Set(1);
					RR_motor.Set(0);
					LR_motor.Set(0);


				}
		if (timr>200 && timr<300)
				{
					LF_motor.Set(0);
					RF_motor.Set(0);
					RR_motor.Set(1);
					LR_motor.Set(0);


				}
		if (timr>300 && timr<400)
				{
					LF_motor.Set(0);
					RF_motor.Set(0);
					RR_motor.Set(0);
					LR_motor.Set(-1);


				}
				*/
/*		if (timr>50)
				{
					LF_motor.Set(0);
					RF_motor.Set(0);
					RR_motor.Set(0);
					LR_motor.Set(0);
				}
		timr++; */

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {
		// ------------------------------------------------------
		// Driving
		// ------------------------------------------------------

		// Used for controlling teleop speed buttons; slow and fast buttons;
		if (controller1.GetRawButton(iJoystickFast_)){
			teleMaxSpeed=1;
		}
		else if (controller1.GetRawButton(iJoystickSlow_)){
			teleMaxSpeed=0.25;
		}
		else
		{
			teleMaxSpeed=0.5;
		}

		// Defines controller input for driving functions
		double joystickX = controller1.GetRawAxis(iJoystickX_);
		double joystickY = controller1.GetRawAxis(iJoystickY_);
		double joystickRot = controller1.GetRawAxis(iJoystickRotate_);

		// If the controller has a super small input this block of code won't count it.
		double NoMove=0.2;
		if (fabs(joystickX) < NoMove){
			joystickX = 0.0;
		}
		if (fabs(joystickY) < NoMove){
			joystickY=0;
		}

		if (fabs(joystickRot) < NoMove){
			joystickRot=0;
		}

		//MecDrive(joystickX, -joystickY, joystickRot);

		// ------------------------------------------------------
		// Shooting
		// ------------------------------------------------------

		// Used for changing the speed on the current shooter, not using closed loop
		ShooterChangeMotorSpeed();
		RF_motor.Set(-shootSpeed);

		// ------------------------------------------------------
		// Gear Arm
		// ------------------------------------------------------

		// Used for controlling the variable for the pneumatic gear arm
		if (controller1.GetRawButton(iJoystickArm))
		{
			armMoving=true;
		}
		else{
			armMoving=false;
		}

		MoveArm(armMoving);

		DisplayRobotStatus();
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
