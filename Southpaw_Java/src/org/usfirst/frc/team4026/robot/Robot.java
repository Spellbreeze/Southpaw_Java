package org.usfirst.frc.team4026.robot;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.hal.PowerJNI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


//import com.ctre.WPI_TalonSRX.TalonControlMode;

/**
 * In changing the iterations of setI, setF, setP, setD to
 * config_kI, etc, I added two values to the parameters:
 * 	0 for slot index
 * 	500 for timeoutMs
 * exp:
 * 	config_kD(0, 0.0, 500)
 */

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at the right
 * time as controlled by the switches on the driver station or the field
 * controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends IterativeRobot {
    static final double PI = 3.14159265;
    static final double GRAVITY_IN_S2 = 385.827;
	static final double SHOOTER_ANGLE_DEGREES = 76;
	static final double SHOOTER_WHEEL_DIAMETER_INCH = 2.375;
	static final double SHOOTER_PCT_EFFICIENCY  = 99.5;
	static final double DRIVE_TICKSPERREV = 64;
	static final double SERVO_UP = 0.2;
	static final double SERVO_DOWN = 1.0; //1.0
	static final boolean USE_DRIVE_TIMER = false;
	static final double MAX_BATTERY = 12.3;
	
	static final int API_MIGRATION_TIMEOUT = 0;
	static final double API_MIGRATION_DOUBLE = 0.0;
	static final int API_MIGRATION_INDEX_SLOT = 0;
	
	//frc::RobotDrive myRobot { 0, 1 }; // robot drive system
	VictorSP rightDriveMotor;
	VictorSP leftDriveMotor;
	WPI_TalonSRX shooterWheelFront;
	WPI_TalonSRX shooterWheelBack;
	WPI_TalonSRX ballIntakeRoller1;
	WPI_TalonSRX ballIntakeRoller2;
	WPI_TalonSRX gearCatcherScrew;
	DoubleSolenoid hanger;
	DoubleSolenoid shifter;
	Compressor compressorPointer;
	Servo shooterServo;
	Servo agitatorServo;
	Talon agitatorMotor;

	Joystick mainDriverStick;
	//Joystick driveRightStick;
	Joystick manipulatorStick;

	AnalogGyro driveGyro;
	AnalogInput wallDistanceSensorS;
	AnalogInput wallDistanceSensorR;
	AnalogInput wallDistanceSensorL;
	DigitalInput photoElectric;
	DigitalInput photoElectricShooter;
	DigitalInput gearCatcherLimitLeft;
	DigitalInput gearCatcherLimitRight;
	Encoder rightDriveEncoder;

	Timer autoDriveTimer;
	Timer agitatorTimer;
	Timer genericTimer;

	boolean stoleDriveTrainControl;	//set to true if an autonomous function is controlling the drive train during tele-op
	boolean stoleDriveTrainControl2; 	//For reverse 3 inch
	boolean driveReverse;
	boolean isGyroresetTelop;
	boolean agitatorUp;
	boolean genericTimerStarted;
	int autoState;
	int gearCatcherState;
	int shootFuelState;
	int driveRevState;
	double avgShooterVelocityError;
	double gyroKi; //Integrator term for gyro
	//const double shootVelocityArray[3] = {1000.0, 3600.0, 4000.0};
	//const double shootDistanceInchArray[3] = {36.0, 118.0, 160.0};
	SendableChooser<String> chooser = new SendableChooser<>();
	String autoNameDefault = "Default";
	String autoNameGear1 = "Gear Location 1";
	String autoNameGear2 = "Gear Location 2";
	String autoNameGear3 = "Gear Location 3";
	String autoNameTwoHopper = "Two Hopper";



	public void robotInit() {
		rightDriveMotor = new VictorSP( 0 );
		leftDriveMotor = new VictorSP(1);
		shooterWheelFront = new WPI_TalonSRX( 1 );
		shooterWheelBack = new WPI_TalonSRX( 5 );
		ballIntakeRoller1 = new WPI_TalonSRX( 2 );
		ballIntakeRoller2 = new WPI_TalonSRX( 4 );
		gearCatcherScrew = new WPI_TalonSRX( 3 );
		hanger = new DoubleSolenoid( 5,2 );
		hanger.set(Value.kReverse);
		shifter = new DoubleSolenoid( 4,3 );
		shifter.set(Value.kReverse);
		shooterServo = new Servo( 4 );
		agitatorServo = new Servo( 5 );
		agitatorMotor = new Talon( 2 );

		mainDriverStick = new Joystick( 0 );
		//driveRightStick = new Joystick ( 1 );
		manipulatorStick = new Joystick( 1 );

		driveGyro = new AnalogGyro( 0 );
		wallDistanceSensorS = new AnalogInput( 3 );
		wallDistanceSensorR = new AnalogInput( 2 );
		wallDistanceSensorL = new AnalogInput( 1 );
		photoElectric = new DigitalInput( 2 );
		photoElectricShooter = new DigitalInput( 3 );
		gearCatcherLimitLeft = new DigitalInput( 1 );
		gearCatcherLimitRight = new DigitalInput( 0 );
		rightDriveEncoder = new Encoder(8 , 9, false);
		
		autoDriveTimer = new Timer();
		agitatorTimer = new Timer();
		genericTimer = new Timer();

		
		stoleDriveTrainControl = false;
		stoleDriveTrainControl2 = false;
		driveReverse = false;
		isGyroresetTelop = false;
		autoState = 0;
		gearCatcherState = 0;
		shootFuelState = 0;
		driveRevState = 0;
		agitatorUp = false;
		genericTimerStarted = false;
		avgShooterVelocityError = 0.0;
		gyroKi = 0.0;
		
		//myRobot.SetExpiration(0.1);

		chooser.addDefault("autoNameDefault", "autoNameDefault");
		chooser.addObject("autoNameGear1", "autoNameGear1");
		chooser.addObject(autoNameGear2, autoNameGear2);
		chooser.addObject(autoNameGear3, autoNameGear3);
		chooser.addObject(autoNameTwoHopper, autoNameTwoHopper);
		SmartDashboard.putData("Auto Modes", chooser);

		//Configure Shooter Talons
		//TODO: need to revisit new pidIdx values instead of autostate
		shooterWheelFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, API_MIGRATION_TIMEOUT);
		
		//TODO: split into two functions, need to figure out which is forward and which is reverse
		shooterWheelFront.configNominalOutputForward(0.0, API_MIGRATION_TIMEOUT);
		shooterWheelFront.configNominalOutputReverse(0.0, API_MIGRATION_TIMEOUT);
		//TODO: split into two functions, need figure out which is forward/reverse
		shooterWheelFront.configPeakOutputForward(0.0, API_MIGRATION_TIMEOUT);  //Modify this to allow for just forward or just backward spin
		shooterWheelFront.configPeakOutputReverse(1.0, API_MIGRATION_TIMEOUT);
		
		//TODO: updated changeControlMode() to set(), fix double
		shooterWheelFront.set(ControlMode.Velocity, API_MIGRATION_DOUBLE);
		//TODO: changed from setInvert to setSensorPhase
		shooterWheelFront.setSensorPhase(false);// SetSensorDirection(false);
		//TODO: fix these parameters for configAllowableClosedloopError
		shooterWheelFront.configAllowableClosedloopError(1, 0, API_MIGRATION_TIMEOUT);
		//TODO: fix slotIdx and pidIdx placeholders
		shooterWheelFront.selectProfileSlot(1, 0);//SelectProfileSlot(0);
		shooterWheelFront.config_kF(1, 0.02497, API_MIGRATION_TIMEOUT); //0.0416
		shooterWheelFront.config_kP(1, 0.0, API_MIGRATION_TIMEOUT);
		shooterWheelFront.config_kI(1, 0.0, API_MIGRATION_TIMEOUT);
		shooterWheelFront.config_kD(1, 0.0, API_MIGRATION_TIMEOUT);
		//shooterWheelFront.set(0.0);
		
		//TODO: revise new values from autostate
		shooterWheelBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, API_MIGRATION_TIMEOUT);
		
		//TODO: split into two functions, figure out forward and reverse
		shooterWheelBack.configNominalOutputForward(0.0, API_MIGRATION_TIMEOUT);
		shooterWheelBack.configNominalOutputReverse(0.0, API_MIGRATION_TIMEOUT);
		//TODO: split into two functions, figure out forward and reverse
		shooterWheelBack.configPeakOutputForward(1.0, API_MIGRATION_TIMEOUT);
		shooterWheelBack.configPeakOutputReverse(0.0, API_MIGRATION_TIMEOUT); //Modify this to allow for just forward or just backward spin
		
		//TODO: updated changeControlMode() to set(), fix double
		shooterWheelBack.set(ControlMode.Velocity, API_MIGRATION_DOUBLE);
		//TODO: double-check proper boolean parameter for setSensorphase (changed from reverseSensor)
		shooterWheelBack.setSensorPhase(false);//SetSensorDirection(false);
		//TODO: fix these slotIdx for this
		shooterWheelBack.configAllowableClosedloopError(5, 0, API_MIGRATION_TIMEOUT);
		//TODO: check for proper slotIdx and pidIdx, 211 was shooterWheelFront
		shooterWheelBack.selectProfileSlot(5, 0);//SelectProfileSlot(0);
		shooterWheelBack.config_kF(5, 0.02497, API_MIGRATION_TIMEOUT);
		shooterWheelBack.config_kP(5, 0.0, API_MIGRATION_TIMEOUT);
		shooterWheelBack.config_kI(5, 0.0, API_MIGRATION_TIMEOUT);
		shooterWheelBack.config_kD(5, 0.0, API_MIGRATION_TIMEOUT);
		//shooterWheelBack.set(0.0);
		hanger.set(Value.kReverse);
		shooterServo.set(0);
		agitatorServo.set(0.9);
		driveReverse = true;
		driveGyro.reset();

		//autoDriveTimer = new Timer();
		//agitatorTimer = new Timer();

		CameraServer.getInstance().startAutomaticCapture();
	}

	/*
	 * Returns true when time in seconds has expired
	 */
	boolean waitAsyncUntil(double timeInSec, boolean resetTimer)
	{
		if(!genericTimerStarted)
		{
			genericTimer.reset();
			genericTimer.start();
			genericTimerStarted = true;
		}

		if(genericTimer.get() >= timeInSec)
		{
			if(resetTimer)
				genericTimerStarted = false;
			return true;
		}

		return false;
	}

	/*
	 * Smooth joystick input for driving
	 */
	double smoothJoyStick(double joyInput)
	{
		return Math.pow(joyInput,2);
	}

	/*
	 * Switch the front and back of the robot
	 */
	/*void toggleDriveDirection()
	{
		if(driveLeftStick.getRawButton(3))
		{
			driveReverse=false;
		}
		else if(driveLeftStick.getRawButton(2))
		{
			driveReverse=true;
		}
	}
	/*
	 * Allow for robot to drive straight using Gyro feedback
	 */
	void keepDriveStraight(double leftDriveVel, double rightDriveVel, double targetAngle)
	{
		double error = 0, correctionFactor;
		error = targetAngle - driveGyro.getAngle();
		correctionFactor = (error/75.0);

		if(leftDriveVel > 0.9)
			leftDriveVel = 0.9;
		else if(leftDriveVel < -0.9)
			leftDriveVel = -0.9;

		if(rightDriveVel > 0.9)
			rightDriveVel = 0.9;
		else if(rightDriveVel < -0.9)
			rightDriveVel = -0.9;

		if(targetAngle > (driveGyro.getAngle() - 0.5) || targetAngle < (driveGyro.getAngle() + 0.5))
		{
			leftDriveMotor.set(((-leftDriveVel) + correctionFactor) * batteryCompensationPct());
			rightDriveMotor.set((rightDriveVel + correctionFactor) * batteryCompensationPct());
		}
		else
		{
			leftDriveMotor.set(-leftDriveVel * batteryCompensationPct());
			rightDriveMotor.set(rightDriveVel * batteryCompensationPct());
		}
	}

	/*
	 * Determine whether we should keep driving straight in tele-op
	 */
	boolean shouldIHelpDriverDriveStraight()
	{
		return false;
	}

	/*
	 * All for human control of drive train
	 */
	void tankDrive()
	{
		//toggleDriveDirection();
		//double right = smoothJoyStick(mainDriverStick.getY());
		//double left = smoothJoyStick(mainDriverStick.getThrottle());
		double right = mainDriverStick.getY();
		double left = mainDriverStick.getThrottle();

		//Cut Velocity in half
		if(mainDriverStick.getRawButton(7))
		{
			right /= 2.0;
			left /= 2.0;
		}

		double avgStick = (right + left) / 2.0;

		if(!mainDriverStick.getRawButton(8) && !shouldIHelpDriverDriveStraight())
		{
			if (driveReverse)
			{
				leftDriveMotor.set(-right);
				rightDriveMotor.set(left);
			}
			else
			{
				leftDriveMotor.set(left);
				rightDriveMotor.set(-right);
			}
			isGyroresetTelop = false;
		}
		else
		{
			if(isGyroresetTelop == false)
			{
				driveGyro.reset();
				isGyroresetTelop = true;
			}
			if (driveReverse)
			{
				keepDriveStraight(avgStick, avgStick, 0);
			}
			else
			{
				keepDriveStraight(-avgStick, -avgStick, 0);
			}
		}
	}

	/*
	 * Spin shooter wheels up to Velocity with door closed.
	 * Return true when wheels are at Velocity
	 */
	boolean spinShooterWheels(double frontWheel, double backWheel)
	{
		//Ensure we do not tell the shooter to spin backwards
		if(frontWheel < 0.0)
			frontWheel = 0.0;
		if(backWheel < 0.0)
			backWheel = 0.0;

		shooterWheelFront.config_kP(0, 0.057, API_MIGRATION_TIMEOUT);
		shooterWheelFront.config_kI(0, 0.0001, API_MIGRATION_TIMEOUT);
		shooterWheelFront.config_kD(0, 1.3, API_MIGRATION_TIMEOUT); //1.2

		shooterWheelBack.config_kP(0, 0.057, API_MIGRATION_TIMEOUT);
		shooterWheelBack.config_kI(0, 0.0001, API_MIGRATION_TIMEOUT);
		shooterWheelBack.config_kD(0, 1.3, API_MIGRATION_TIMEOUT);

		shooterWheelFront.set(-1.0 * frontWheel);
		shooterWheelBack.set(backWheel);
		
		avgShooterVelocityError = (shooterWheelFront.getClosedLoopError(0) + shooterWheelBack.getClosedLoopError(0)) / 2.0;

		if(avgShooterVelocityError < 200 && (shooterWheelBack.getSelectedSensorVelocity(0) > (backWheel * 0.9))) //500
			return true;

		return false;
	}

	/*
	 * Stop shooter wheels from spinning
	 */
	void stopShooter()
	{
		shooterServo.set(SERVO_DOWN);

		shooterWheelFront.config_kP(0, 0.0, API_MIGRATION_TIMEOUT);
		shooterWheelFront.config_kI(0, 0.0, API_MIGRATION_TIMEOUT);
		shooterWheelFront.config_kD(0, 0.0, API_MIGRATION_TIMEOUT);
		shooterWheelBack.config_kP(0, 0.0, API_MIGRATION_TIMEOUT);
		shooterWheelBack.config_kI(0, 0.0, API_MIGRATION_TIMEOUT);
		shooterWheelBack.config_kD(0, 0.0, API_MIGRATION_TIMEOUT);

		shooterWheelFront.set(0.0);
		shooterWheelBack.set(0.0);

		agitatorMotor.set(1.0);
	}

	/*
	 * Perform task of shooting fuel
	 */
	
	//TODO: I think the error stopping the bot from shooting is here.
	void shootFuel(boolean useDistanceSensor, double frontVel, double backVel)
	{
		if(useDistanceSensor)
		{	//double shootRPM = calculateShotVelocityBasedOnDistance();

			if(frontVel != 0.0) //0.0 indicated error
			{
				if(spinShooterWheels(frontVel, backVel))
					shooterServo.set(SERVO_UP);
				else
					shooterServo.set(SERVO_DOWN);
			}
		}
		else
		{
			if(spinShooterWheels(frontVel, backVel)) //3400, 3500
				shooterServo.set(SERVO_UP);
			else
				shooterServo.set(SERVO_DOWN);
		}

		if(!agitatorUp && (agitatorTimer.get() > 1.0))
		{
			agitatorServo.set(0.2);
			agitatorMotor.set(-1.0);
			agitatorTimer.reset();
			agitatorTimer.start();
			agitatorUp = true;
		}
		else if(agitatorTimer.get() > 2.0)
		{
			agitatorServo.set(0.9);
			agitatorMotor.set(1.0);
			agitatorTimer.reset();
			agitatorTimer.start();
			agitatorUp = false;
		}
	}

	/*
	 * Simple manual shooter control.  A more complex shooting scheme will be needed to step through opening the door and Waiting until the shooter is at Velocity.
	 */
	void shootFuelControl()
	{
		double angleBoilerFoundDeg = 0.0;
		double sVel = 0.0;

		if(manipulatorStick.getY() > 0.1 || manipulatorStick.getY() < -0.1)
		{
			/*
			if(spinShooterWheels(manipulatorStick.getY() * 3600.0, manipulatorStick.getY() * 3400.0))
				shooterServo.set(SERVO_UP);
			else
				shooterServo.set(SERVO_DOWN);
			 */
		}
		else if(manipulatorStick.getRawButton(1))
		{
			stoleDriveTrainControl = true;
			switch(shootFuelState)
			{
				case 0:
					resetDrive(USE_DRIVE_TIMER);
					shootFuelState++;
					break;

				case 1:
					//Search for boiler turning max 90 deg clockwise (assuming gear catcher is the front)
					if(turnGyro(-90.0, 0.3))
					{
						Timer.delay(0.25);
						resetDrive(USE_DRIVE_TIMER);
						shootFuelState = -1;
					}
					if(!photoElectricShooter.get())
					{
						stopRobotDrive();

						//angleBoilerFoundDeg = driveGyro.getAngle();
						sVel = calculateShotVelocityBasedOnDistance();

						agitatorUp = false;
						agitatorTimer.reset();
						agitatorTimer.start();

						shootFuelState++;
					}
					break;

				case 2:
					//Attempt to keep the robot pointing in the correct direction
					if(!photoElectricShooter.get()) //&& turnGyro(angleBoilerFoundDeg, 0.3)
					{
						if(manipulatorStick.getRawButton(2))
						{
							shootFuel(false, 3200.0, 3200.0); //Use the distance sensor to adjust shot Velocity set to true
						}
						else
						{
							shootFuel(true, sVel, sVel);
						}
					}
					else
					{
						//Robot lost sight of the boiler with the photoelectric sensor so stop shooting
						stopShooter();
						//shootFuelState = -1;
					}
					break;

				default:
					stopRobotDrive();
					stopShooter();
					break;
			}
		}
		else if(manipulatorStick.getRawButton(2))
		{
			shootFuel(false, 3200.0, 3200.0);	//For manual emergency
		}
		else
		{
			//Stop shooting
			stopShooter();

			shootFuelState = 0;
			stoleDriveTrainControl = false;

			agitatorUp = false;
			agitatorTimer.reset();
			agitatorTimer.start();
		}
	}

	/*
	 * Simple manual gear catcher control using manipulator stick X axis
	 */
	void controlGearCatcher()
	{
		//Should set a dead-zone for this despite the Velocity controllers having one built in
		//gearCatcherScrew.set(manipulatorStick.getX());
		if(manipulatorStick.getRawButton(3))
		{
			findGearCatcherLift();
		}
		else
		{
			if(manipulatorStick.getRawButton(5) && gearCatcherLimitLeft.get())
			{
				gearCatcherScrew.set(0.7);
			}
			else if(manipulatorStick.getRawButton(6) && gearCatcherLimitRight.get())
			{
				gearCatcherScrew.set(-0.7);
			}
			else
			{
				gearCatcherScrew.set(0.0);
			}

			gearCatcherState = 0;
		}
		if(manipulatorStick.getRawButton(4)){
			hanger.set(Value.kForward);
		}
		else
		{
			hanger.set(Value.kReverse);
		}
	}

	/*
	 * Simple manual intake control (doubles as robot climb control for the moment)
	 */
	void controlBallIntake()
	{
		if(manipulatorStick.getRawButton(8))
		{
			ballIntakeRoller1.set(-1.0);
			ballIntakeRoller2.set(-1.0);
		}
		else if(manipulatorStick.getRawButton(7))
		{
			ballIntakeRoller1.set(-0.3);
			ballIntakeRoller2.set(-0.3);
		}
		else
		{
			ballIntakeRoller1.set(0.0);
			ballIntakeRoller2.set(0.0);
		}
	}

	/*
	 * Stop the drive motors
	 */
	void stopRobotDrive()
	{
		leftDriveMotor.set(0);
		rightDriveMotor.set(0);
	}

	/*
	 * For use during autonomous.  Call before initiating an automated drive command.
	 */
	void resetDrive(boolean isTimerBased)
	{
		if(isTimerBased)
		{
			autoDriveTimer.reset();
			autoDriveTimer.start();
			resetGyroAngle();
		}
		else
		{
			//leftDriveEncoder.reset();
			rightDriveEncoder.reset();
			resetGyroAngle();
		}
	}

	/*
	 * reset the Gyro position
	 */
	void resetGyroAngle()
	{
		driveGyro.reset();
	}

	/*
	 * Perform the conversion of encoder ticks to inches
	 */
	double convertDriveTicksToInches(int encTicks)
	{
		return (encTicks / DRIVE_TICKSPERREV) * 3.14 * 4.0;
	}

	/*
	 * Used during autonomous to turn the robot to a specified angle.
	 */
	boolean turnGyro(double rAngle, double maxTurnVelocity)
	{	 ;
		
		double error = 0.0;
		double VelocityToSet = 0.0;
		//Positive gyro angle means turning left
		if(rAngle < driveGyro.getAngle())
		{
			//Start accumulating error if the rate of turning is < 2 deg/sec
			if(driveGyro.getRate() < 2.0)
			{
				gyroKi += 0.001;
				if(gyroKi > 0.2) //Cap the integral term
					gyroKi = 0.2;
			}

			error = Math.abs(rAngle) - driveGyro.getAngle();
			if(driveGyro.getAngle() <= Math.abs(rAngle) && Math.abs(error) > 2.0)
			{
				//turn left
				VelocityToSet = (error/270) + 0.2 + gyroKi; //140 0.2
				if(Math.abs(VelocityToSet) > maxTurnVelocity)
					VelocityToSet = maxTurnVelocity * (VelocityToSet < 0.0 ? -1.0 : 1.0);
				leftDriveMotor.set(VelocityToSet * batteryCompensationPct()); //0.8
				rightDriveMotor.set(VelocityToSet * batteryCompensationPct()); //0.8
			}
			else
			{
				gyroKi = 0.0;
				stopRobotDrive();
				//if(WaitAsyncUntil(0.5,true))
					return true;
			}
		}
		else if(rAngle > driveGyro.getAngle())
		{
			//Start accumulating error if the rate of turning is < 2 deg/sec
			if(driveGyro.getRate() < 2.0)
			{
				gyroKi += 0.001;
				if(gyroKi > 0.2) //Cap the integral term
					gyroKi = 0.2;
			}

			error = -rAngle - driveGyro.getAngle();
			if(driveGyro.getAngle() >= -rAngle && Math.abs(error) > 2.0)
			{
				//turn right
				VelocityToSet = (error/270) - 0.2 - gyroKi;
				if(Math.abs(VelocityToSet) > maxTurnVelocity)
					VelocityToSet = maxTurnVelocity * (VelocityToSet < 0.0 ? -1.0 : 1.0);
				leftDriveMotor.set(VelocityToSet * batteryCompensationPct()); //-0.8
				rightDriveMotor.set(VelocityToSet * batteryCompensationPct()); //-0.8
			}
			else
			{
				gyroKi = 0.0;
				stopRobotDrive();
				//if(WaitAsyncUntil(0.5,true))
					return true;
			}
		}
		else
		{
			gyroKi = 0.0;
			stopRobotDrive();
			return true;
		}

		return false;
	}

	/*
	 * Used during autonomous to drive the robot fwd or back to a location
	 * Prior to calling this function you must call resetDrive
	 */
	boolean autoDriveRobot(double velocityLeft, double velocityRight, double timeSec, double targetDistanceInch, boolean isTimerBased)
	{
		double err = 0.0;
		double driveDistInch = 0.0;
		double percentPower = 0.0;
		if(isTimerBased)
		{
			if(autoDriveTimer.get() <= timeSec)
			{
				//leftDriveMotor.set(-velocityLeft);
				//rightDriveMotor.set(velocityRight);
				keepDriveStraight(velocityLeft, velocityRight, 0);
			}
			else
			{
				stopRobotDrive();
				return true;
			}
		}
		else
		{
			driveDistInch = Math.abs(convertDriveTicksToInches(rightDriveEncoder.get()));
			if(driveDistInch < Math.abs(targetDistanceInch))
			{
				//leftDriveMotor.set(-velocityLeft);
				//rightDriveMotor.set(velocityRight);
				err = Math.abs(targetDistanceInch) - driveDistInch;
				percentPower = (err / Math.abs(targetDistanceInch));

				if(err <= 48.0)	//If within 24" start slowing down
				{
					velocityLeft *= percentPower;
					velocityRight *= percentPower;

					if(velocityLeft < 0.0 && velocityLeft > -0.2)
						velocityLeft = -0.2;
					else if(velocityLeft > 0.0 && velocityLeft < 0.2)
						velocityLeft = 0.2;
					if(velocityRight < 0.0 && velocityRight > -0.2)
						velocityRight = -0.2;
					else if(velocityRight > 0.0 && velocityRight < 0.2)
						velocityRight = 0.2;
				}

				keepDriveStraight(velocityLeft, velocityRight, 0);
			}
			else
			{
				stopRobotDrive();
				return true;
			}
		}
		return false;
	}

	/*
	 * Calculate the shooter Velocity based on ultrasonic measured distance
	 */
	double calculateShotVelocityBasedOnDistance()
	{
		double currentShooterDistanceInch = CalculateWallDistanceShooter(false) + 17.5;

		//Minimum shot distance
		if(currentShooterDistanceInch < 24.0)
			return 0.0;

		double shooterVelocity = 0.0;
		double shooterCalculatedRPM = 0.0;
		shooterVelocity = Math.sqrt(((currentShooterDistanceInch * 2.0) * GRAVITY_IN_S2) / Math.sin(2.0 * SHOOTER_ANGLE_DEGREES * PI / 180.0));
		shooterCalculatedRPM = (shooterVelocity * 60.0)/ (SHOOTER_WHEEL_DIAMETER_INCH * PI) * (SHOOTER_PCT_EFFICIENCY / 100.0);

		SmartDashboard.putNumber("Calculated Shot Velocity (RPM): ", shooterCalculatedRPM);

		return shooterCalculatedRPM;
	}

	/*
	 * Used to calculate the robot distance from the wall
	 */
	double CalculateWallDistanceShooter(boolean averaged)
	{	
		double rawVoltage;
		double wallDistance;

		if(averaged)
			rawVoltage = (double)(wallDistanceSensorS.getAverageVoltage());
		else
			rawVoltage = (double)(wallDistanceSensorS.getVoltage());

		//MB1030
		double VFiveMM = 0.009671875;
		wallDistance = rawVoltage / VFiveMM;

		return wallDistance;
	}

	/*
	 * Used to calculate the robot distance from the wall
	 */
	double CalculateWallDistanceR(boolean averaged)
	{	 
	
		double rawVoltage;
		double wallDistance;

		if(averaged)
			rawVoltage = (double)(wallDistanceSensorR.getAverageVoltage());
		else
			rawVoltage = (double)(wallDistanceSensorR.getVoltage());

		//MB1013
		double VFiveMM = 0.00488; //Old numbers 0.0048359375;  //((4.952 / 5120) * 5);
		wallDistance = ((rawVoltage * 5 * 0.0393) / VFiveMM);  //Units inch

		//MB1030
		//double VFiveMM = 0.009671875;
		//wallDistance = rawVoltage / VFiveMM;

		return wallDistance;
	}

	/*
	 * Used to calculate the robot distance from the wall
	*/
	double CalculateWallDistanceL(boolean averaged)
	{	
		double rawVoltage;
		double wallDistance;

		if(averaged)
			rawVoltage = (double)(wallDistanceSensorL.getAverageVoltage());
		else
			rawVoltage = (double)(wallDistanceSensorL.getVoltage());

		//MB1013
		double VFiveMM = 0.00488; //Old numbers 0.0048359375;  //((4.952 / 5120) * 5);
		wallDistance = ((rawVoltage * 5 * 0.0393) / VFiveMM);  //Units inch

		return wallDistance;
	}

	/*
	 * Calculate a battery compensation percentage to multiply pwm output by
	 */
	double batteryCompensationPct()
	{
		double batteryScaleFactor = 0.0;

		// RobotController.getInstance().getBatteryVoltage issue https://www.chiefdelphi.com/forums/showthread.php?p=1717817
		batteryScaleFactor = MAX_BATTERY / PowerJNI.getVinVoltage();

		return batteryScaleFactor;
	}

	/*
	 * Update the SmartDashboard
	 */
	void updateDashboard()
	{
		//SmartDashboard::PutNumber("Wall Distance Right: ", CalculateWallDistanceR(false));
		SmartDashboard.putNumber("Wall Distance Right: ", shooterWheelBack.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Wall Distance Left: ", CalculateWallDistanceL(false));
		SmartDashboard.putNumber("Wall Distance Shooter: ", CalculateWallDistanceShooter(false));
		SmartDashboard.putNumber("Gyro Reading: ", driveGyro.getAngle());

		SmartDashboard.putBoolean("GearLimitLeft: ", gearCatcherLimitLeft.get());
		SmartDashboard.putBoolean("GearLimitRight: ", gearCatcherLimitRight.get());
		SmartDashboard.putBoolean("photoElectric: ", photoElectric.get());
		SmartDashboard.putBoolean("photoElectricShooter: ", photoElectricShooter.get());

		SmartDashboard.putNumber("Avg Shooter Vel Error: ", avgShooterVelocityError);

		SmartDashboard.putNumber("Drive Encoder Ticks: ", rightDriveEncoder.get());
		SmartDashboard.putNumber("Drive Encoder Inch: ", convertDriveTicksToInches(rightDriveEncoder.get()));

		SmartDashboard.putNumber("Battery Scaling Factor: ", batteryCompensationPct());
		SmartDashboard.putNumber("Voltage", PowerJNI.getVinVoltage()); //I wrote this
	}

	/*
	 * Align robot parallel with wall
	 */
	double performRobotFaceAlignment()
	{
		double angleToTurnDegrees = 0.0;
		try
		{
			angleToTurnDegrees = (Math.atan((CalculateWallDistanceR(false) - CalculateWallDistanceL(false)) / 22.625) * 180.0) / PI;
		}
		catch(Exception e)
		{
			angleToTurnDegrees = 0.0;
		}
		SmartDashboard.putNumber("Angle Off Parallel: ", angleToTurnDegrees);

		return angleToTurnDegrees;
	}

	/*
	 * Automatically find the retro-reflective tape for the gear lift
	 */
	boolean findGearCatcherLift()
	{

		switch(gearCatcherState)
		{
			case 0:
				if (gearCatcherLimitRight.get()){
					gearCatcherScrew.set(-0.7);
				}
				else
				{
					gearCatcherScrew.set(0.0);
					gearCatcherState++;
				}
				break;
			case 1:
				if (photoElectric.get() && gearCatcherLimitLeft.get())
				{
					gearCatcherScrew.set(0.5); //0.4
				}
				else
				{
					gearCatcherScrew.set(0.0);

					if(!gearCatcherLimitLeft.get())
						gearCatcherState = 0;
					else
						return true;
						//gearCatcherState++;
				}
				break;
			case 2:
				if (!photoElectric.get())
				{
					gearCatcherScrew.set(0.0);
					return true;
				}
				else
				{
					gearCatcherState = 0;
				}
				break;

			default:
				gearCatcherScrew.set(0.0);
				break;
		}

		return false;
	}

	/*
	 * Do nothing
	 */
	void doNothingAutonomous()
	{
		switch(autoState)
		{
			default:
				stopRobotDrive();
				break;
		}
	}

	/*
	 * get both middle and close hopper of balls (RED)
	 * I've assumed that negative angles will turn clockwise relative to the gear catcher being the front
	 * I've assume positive drive motor Velocitys will drive the robot in reverse (relative to the gear catcher being the front)
	 */
	void score_TwoHopper_Autonomous(boolean isRED)
	{
		double sVel = 0.0;
		double angleMultiplier = 1.0;
		double angleToSearchForBoiler = -60.0;
		double closeHopperDistance = 84.0 + 34.0; //Red Alliance 84.0 52.0
		if(!isRED)
		{
			//Blue
			angleMultiplier = -1.0;
			closeHopperDistance = 84.0;
			angleToSearchForBoiler = -100.0;
		}

		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 1:
				//Drive the robot in reverse to get to middle hopper
				if(autoDriveRobot(0.5, 0.5, 0, closeHopperDistance, USE_DRIVE_TIMER))
				{
					Timer.delay(0.25);
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 2:
				//Turn intake side towards hopper
				if(turnGyro(-90.0 * angleMultiplier, .5))
				{
					//Wait(0.25);
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 3:
				//Drive the robot reverse to trigger hopper
				if(autoDriveRobot(0.6, 0.6, 1.5, 84, USE_DRIVE_TIMER) || waitAsyncUntil(2.0, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 4:
				//Wait some time for the hopper to empty into robot
				if(waitAsyncUntil(1.0, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 5:
				//Drive the robot forward away from hopper
				if(autoDriveRobot(-0.5, -0.5, 0, 36, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 6:
				//Turn to start position for searching
				if(!isRED)
				{
					//BLUE
					if(turnGyro(-160.0 , .5))
					{
						Timer.delay(0.25);
						resetDrive(USE_DRIVE_TIMER);
						autoState++;
					}
				}
				else
				{
					//RED
					if(turnGyro(70.0 , .5)) //60
					{
						Timer.delay(0.25);
						resetDrive(USE_DRIVE_TIMER);
						autoState++;
					}
				}
				break;
			case 7:
				//Drive the robot forward away from hopper
				if(isRED)
				{
					if(autoDriveRobot(-0.5, -0.5, 0, 34, USE_DRIVE_TIMER))  //36
					{
						resetDrive(USE_DRIVE_TIMER);
						autoState++;
					}
				}
				else
				{
					autoState++;
				}
				break;
			case 8:
				if(turnGyro(angleToSearchForBoiler, 0.3))
				{
					Timer.delay(0.25);
					resetDrive(USE_DRIVE_TIMER);
					autoState = -1;
				}
				if(!photoElectricShooter.get())
				{
					stopRobotDrive();

					//sVel = calculateShotVelocityBasedOnDistance();

					//agitatorUp = false;
					//agitatorTimer.reset();
					//agitatorTimer.Start();

					autoState++;
				}
				break;
			case 9:
				//Turn until not seeing boiler
				if(turnGyro(4.0, 0.3))
				{
					Timer.delay(0.25);
					resetDrive(USE_DRIVE_TIMER);

					stopRobotDrive();

					sVel = calculateShotVelocityBasedOnDistance();

					agitatorUp = false;
					agitatorTimer.reset();
					agitatorTimer.start();
					autoState++;
				}
				break;
			case 10:
				shootFuel(true, sVel, sVel);
				if(waitAsyncUntil(7.0, true))
				{
					stopShooter();
					resetDrive(USE_DRIVE_TIMER);
					//autoState++;
					autoState = -1;
				}
				break;

			default:
				stopRobotDrive();
				break;
		}
	}

	void takeOverDrive()
	{

			stoleDriveTrainControl2 = false;
			driveRevState = 0;

	}

	void reverseNInches(double driveDistanceToReverse)
	{
		switch(driveRevState)
				{
					case 0:
						resetDrive(USE_DRIVE_TIMER);
						driveRevState++;
						break;
					case 1:
						//Drive the robot in reverse
						if(autoDriveRobot(0.25, 0.25, 0, driveDistanceToReverse, USE_DRIVE_TIMER))
						{
							resetDrive(USE_DRIVE_TIMER);
							driveRevState++;
						}
						break;

					default:
						stoleDriveTrainControl2 = false;
						stopRobotDrive();
						break;
				}
	}

	/*
	 * Score gear on peg RED (Gear position 1 is closest to the boiler)
	 * I've assumed that negative angles will turn clockwise relative to the gear catcher being the front
	 * I've assume positive drive motor Velocitys will drive the robot in reverse (relative to the gear catcher being the front)
	 */
	void score_GearPosition1_Autonomous(boolean isRED, boolean shootFuelAfterGear)
	{
		double angleErrorFromUltrasonics = 0.0;
		double angleToTurn = -113.0; //For RED -120
		double angleForBoiler = 110.0; //90
		double distanceToDrive = 76.0; //For RED. was 72
		double distanceForGearPlacement = 36; //30
		double sVel = 0.0;

		if(!isRED)
		{
			angleToTurn = 116.0; //112
			distanceToDrive = 88.0; //73
			distanceForGearPlacement = 36;
		}

		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 1:
				//Drive the robot in reverse
				if(autoDriveRobot(0.5, 0.5, 0, distanceToDrive, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
					//autoState= -1;
				}
				break;
			case 2:
				if(turnGyro(angleToTurn, 0.35))
				{
					Timer.delay(0.25);
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 3:
				//Home gear catcher
				gearCatcherState = 0;
				findGearCatcherLift();

				//Drive the robot forward to get a bit closer to airship
				if(autoDriveRobot(-0.4, -0.4, 1.3, 20, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					//autoState++;
					autoState = 6;//6
				}
				break;
			case 4:
				//Align the robot face parallel to airship wall
				angleErrorFromUltrasonics = performRobotFaceAlignment();
				if(Math.abs(angleErrorFromUltrasonics) > 20.0)	//Assume that if the angle returned is a large value there has been an error with the sensors
				{
					autoState = -1;	//Abort
				}
				else
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 5:
				if(turnGyro(angleErrorFromUltrasonics , .5))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 6:
				//Search for the peg using the photoelectric sensor
				if(findGearCatcherLift())
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				/*else if(WaitAsyncUntil(5.0, true))
				{
					autoState = -1; //Abort due to timeout
				}*/
				break;
			case 7:
				//Drive the robot forward to get the gear on the peg
				if(autoDriveRobot(-0.3, -0.3, 0.5, distanceForGearPlacement, USE_DRIVE_TIMER) || waitAsyncUntil(2.0, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 8:
				//Wait some time for the pilot to remove the gear.  Would be better to have a sensor for this.
				if(waitAsyncUntil(2.0, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 9:
				//Drive the robot reverse
				if(autoDriveRobot(0.4, 0.4, 0.6, 26, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);

					if(shootFuelAfterGear)
						autoState++;
					else
						autoState = 6;
				}
				break;
			case 10:
				if(turnGyro(angleForBoiler , .5))
				{
				Timer.delay(0.25);

					//agitatorUp = false;
					//agitatorTimer.reset();
					//agitatorTimer.Start();

					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 11:
				if(turnGyro(-1.0 * angleForBoiler, 0.3))
				{
					Timer.delay(0.25);
					resetDrive(USE_DRIVE_TIMER);
					autoState = -1;
				}
				if(!photoElectricShooter.get())
				{
					stopRobotDrive();
					sVel = calculateShotVelocityBasedOnDistance();

					agitatorUp = false;
					agitatorTimer.reset();
					agitatorTimer.start();

					autoState++;
				}
				break;
			case 12:
				shootFuel(true, sVel, sVel);
				if(waitAsyncUntil(5.0, true))
				{
					stopShooter();
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			default:
				stopRobotDrive();
				break;
		}
	}

	/*
	 * Score gear on peg RED (Gear position 2 is middle peg)
	 * I've assumed that negative angles will turn clockwise relative to the gear catcher being the front
	 * I've assume positive drive motor Velocitys will drive the robot in reverse (relative to the gear catcher being the front)
	 */
	void score_GearPosition2_Autonomous(boolean isRED, boolean useUltra)
	{
		double distanceToDrive = 54.0; //44
		double angleForBoiler = -15.0; //RED -20
		double sVel = 0.0;

		if(!isRED)
		{
			//BLUE
			angleForBoiler = 15.0;
		}

		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				autoState++;
				break;
			case 1:
				//Home gear catcher
				gearCatcherState = 0;
				findGearCatcherLift();

				//Drive the robot forward
				if(autoDriveRobot(-0.5, -0.5, 0, distanceToDrive, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
					//autoState= -1;
				}
				break;
			case 2:
				//Search for the peg using the photoelectric sensor
				if(findGearCatcherLift())
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				/*else if(WaitAsyncUntil(5.0, true))
				{
					autoState = -1; //Abort due to timeout
				}*/
				break;
			case 3:
				//Drive the robot forward to get the gear on the peg
				if(autoDriveRobot(-0.3, -0.3, 0.5, 36, USE_DRIVE_TIMER) || waitAsyncUntil(1.5, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 4:
				//Wait some time for the pilot to remove the gear.  Would be better to have a sensor for this.
				if(waitAsyncUntil(3.0, true))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 5:
				//Drive the robot reverse
				if(autoDriveRobot(0.4, 0.4, 0.6, 24, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 6:
				if(isRED)
				{
					if(turnGyro(-180.0 , .5))
					{
						resetDrive(USE_DRIVE_TIMER);
						autoState++;
					}
				}
				else
				{
					autoState++;
				}
				break;
			case 7:
				if(turnGyro(angleForBoiler, 0.3))
				{
					Timer.delay(0.25);
					resetDrive(USE_DRIVE_TIMER);

					stopRobotDrive();

					sVel = calculateShotVelocityBasedOnDistance();

					agitatorUp = false;
					agitatorTimer.reset();
					agitatorTimer.start();

					autoState++;
				}
				break;
			case 8:
				if(useUltra)
				{
					shootFuel(true, sVel, sVel);
				}
				else
				{
					shootFuel(false, 4250.0, 4250.0); //3900
				}
				if(waitAsyncUntil(5.0, true))
				{
					stopShooter();
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;

			default:
				stopRobotDrive();
				break;
		}
	}


	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void Autonomous() {
		DriverStation.Alliance allianceColor;
		boolean isAllianceRED = false;
		Object autoSelected = chooser.getSelected();
		// std::string autoSelected = frc::SmartDashboard::getString("Auto Selector", autoNameDefault);
		System.out.println( "Auto selected: " + autoSelected.toString());

		gearCatcherState = 0;
		autoState = 0;

		//Determine current alliance color
		allianceColor = DriverStation.getInstance().getAlliance();
		if(allianceColor == DriverStation.Alliance.Red)
		{
			isAllianceRED = true;
		}
		updateDashboard();

		while (isAutonomous() && isEnabled())
		{
			if(autoSelected == autoNameGear1)
			{
				score_GearPosition1_Autonomous(isAllianceRED, true); //set second param to false to retry gear
			}
			else if (autoSelected == autoNameGear2)
			{
				score_GearPosition2_Autonomous(isAllianceRED, true);
			}
			else if (autoSelected == autoNameGear3)
			{
				score_GearPosition1_Autonomous(!isAllianceRED, false);
			}
			else if (autoSelected == autoNameTwoHopper)
			{
				score_TwoHopper_Autonomous(isAllianceRED);
			}
			else
			{
				//Default Auto goes here
				//score_GearPosition2_Autonomous();
				//score_GearPosition1_Autonomous(isAllianceRED, true);
				doNothingAutonomous();
			}
			updateDashboard();
			Timer.delay(0.005);				// Wait for a motor update time
		}
	}

	/*
	 * Runs the motors with arcade steering.
	 */
	public void teleopPeriodic() {
		//myRobot.SetSafetyEnabled(true);
		driveGyro.reset();
		while (isOperatorControl() && isEnabled())
		{
			if(!stoleDriveTrainControl && !stoleDriveTrainControl2)
				tankDrive();
			shootFuelControl();
			controlGearCatcher();
			controlBallIntake();
			takeOverDrive();
			updateDashboard();

			calculateShotVelocityBasedOnDistance();
			// Wait for a motor update time
			Timer.delay(0.005);
			
		}
	}

	/*
	 * Runs during test mode
	 */
	void Test() {

	}
}