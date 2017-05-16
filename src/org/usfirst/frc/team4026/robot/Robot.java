package org.usfirst.frc.team4026.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	VictorSP leftDriveMotor, rightDriveMotor;
	boolean exitvalue;
	CANTalon shooterWheelFront;
    CANTalon shooterWheelBack;
    CANTalon ballIntakeRoller1;
    CANTalon ballIntakeRoller2;
    CANTalon gearCatcherScrew;
    Servo shooterServo;
   	Servo agitatorServo;
    Talon agitatorMotor;
    
    Joystick driveLeftStick;
    Joystick driveRightStick;
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
	
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	boolean stoleDriveTrainControl;	//Set to true if an autonomous function is controlling the drive train during tele-op
	boolean stoleDriveTrainControl2; 	//For reverse 3 inch
	boolean driveReverse;
	boolean isGyroResetTelop;
	boolean agitatorUp;
	boolean genericTimerStarted;
	int autoState;
	int gearCatcherState;
	int shootFuelState;
	int driveRevState;
	double avgShooterVelocityError;
	double gyroKi; //Integrator term for gyro
	double pi = 3.14159265;
	double gravity_in_S2 = 385.827;
	int shooter_Angle_Degrees = 76;
	double shooter_Wheel_Diameter_Inch = 2.375;
	double shooter_PCT_Efficiency = 99.5;
	int drive_TicksPerRev = 64;
	double servo_Up = .2;
	double servo_Down = 1.0;
	boolean use_Drive_Timer;
	double max_Battery = 12.3;
	static final String autoNameDefault = "Default";
	static final String autoNameGear1 = "Gear Location 1";
	static final String autoNameGear2 = "Gear Location 2";
	static final String autoNameGear3 = "Gear Location 3";
	static final String autoNameTwoHopper = "Two Hopper";
	//stoleDriveTrainControl = false;
	//stoleDriveTrainControl2 = false;
	//driveReverse = false;
	//isGyroResetTelop = false;
	//autoState = 0;
	//gearCatcherState = 0;
	//shootFuelState = 0;
	//driveRevState = 0;
	//agitatorUp = false;
	//genericTimerStarted = false;
	//avgShooterVelocityError = 0.0;
	//gyroKi = 0.0;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", autoNameDefault);
		chooser.addObject("Gear Location 1", autoNameGear1);
		chooser.addObject("Gear Location 2", autoNameGear2);
		chooser.addObject("Gear Location 3", autoNameGear3);
		chooser.addObject("Two Hopper", autoNameTwoHopper);
		SmartDashboard.putData("Auto choices", chooser);
		
		rightDriveMotor = new VictorSP(0);
		leftDriveMotor = new VictorSP(1);
		
		shooterWheelFront = new CANTalon(1);
		shooterWheelBack = new CANTalon(5);
		ballIntakeRoller1 = new CANTalon(2);
		ballIntakeRoller2 = new CANTalon(4);
		gearCatcherScrew = new CANTalon(3);
		
		agitatorServo = new Servo(5);
		shooterServo = new Servo(4);
		
		agitatorMotor = new Talon(2);
		
		wallDistanceSensorS = new AnalogInput(3);
		wallDistanceSensorR = new AnalogInput(2);
		wallDistanceSensorL = new AnalogInput(1);
		
		gearCatcherLimitLeft = new DigitalInput(1);
		gearCatcherLimitLeft = new DigitalInput(0);
		photoElectric = new DigitalInput(2);
		photoElectricShooter = new DigitalInput(3);
		driveGyro = new AnalogGyro(0);
		rightDriveEncoder = new Encoder(8,9,false);
		
		myRobot = new RobotDrive(leftDriveMotor, rightDriveMotor);
       
	    driveLeftStick = new Joystick(0);
        driveRightStick = new Joystick(1);
        manipulatorStick = new Joystick(2);
        
        autoDriveTimer = new Timer();
    	agitatorTimer = new Timer();
    	genericTimer = new Timer();
    	
    	//Configure Shooter Talons
   		shooterWheelFront.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
   		shooterWheelFront.configNominalOutputVoltage(+0.0f, -0.0f);
    	shooterWheelFront.configPeakOutputVoltage(+0.0f, -12.0f);  //Modify this to allow for just forward or just backward spin
    	shooterWheelFront.changeControlMode(CANTalon.TalonControlMode.Speed);
    	shooterWheelFront.reverseSensor(false);//idk if this equals c++ setSensorDirection
    	shooterWheelFront.setAllowableClosedLoopErr(0);
    	shooterWheelFront.setProfile(0);//idk if this equals c++ selectProfileSlot(0);
    	shooterWheelFront.setF(0.02497); //0.0416
    	shooterWheelFront.setP(0.0);
    	shooterWheelFront.setI(0.0);
    	shooterWheelFront.setD(0.0);
    	//shooterWheelFront.Set(0.0);


		shooterWheelBack.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
		shooterWheelBack.configNominalOutputVoltage(+0.0f, -0.0f);
		shooterWheelBack.configPeakOutputVoltage(+12.0f, -0.0f); //Modify this to allow for just forward or just backward spin
		shooterWheelBack.changeControlMode(CANTalon.TalonControlMode.Speed);
		shooterWheelBack.reverseSensor(false);//idk if this equals c++ setSensorDirection
		shooterWheelBack.setAllowableClosedLoopErr(0);
		shooterWheelBack.setProfile(0);//idk if this equals c++ selectProfileSlot(0);
		shooterWheelBack.setF(0.02497);
		shooterWheelBack.setP(0.0);
		shooterWheelBack.setI(0.0);
		shooterWheelBack.setD(0.0);
		//shooterWheelBack.Set(0.0);

		shooterServo.set(1.0);
		agitatorServo.set(0.9);
		driveReverse = true;
		driveGyro.reset();

		//autoDriveTimer = new Timer();
		//agitatorTimer = new Timer();

		CameraServer.getInstance().startAutomaticCapture();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/*
	 * Returns true when time in seconds has expired
	 */
	boolean WaitAsyncUntil(double timeInSec, boolean resetTimer)
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
	double smoothJoyStick(float joyInput)
	{
		return Math.pow(joyInput,2);
	}

	/*
	 * Switch the front and back of the robot
	 */
	void toggleDriveDirection()
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
		double error = 0;
		double correctionFactor;
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
		double right = driveRightStick.getY();
		double left = driveLeftStick.getY();
		double diff = Math.abs(right-left);
		boolean sameSign = ((right < 0.0 && left < 0.0) || (right > 0.0 && left > 0.0))  ? true : false;

		if(sameSign && (diff < 0.2))
			return true;

		return false;
	}

	/*
	 * All for human control of drive train
	 */
	void tankDrive()
	{
		toggleDriveDirection();
		//double right = smoothJoyStick(driveRightStick.GetY());
		//double left = smoothJoyStick(driveLeftStick.GetY());
		double right = driveRightStick.getY();
		double left = driveLeftStick.getY();

		//Cut speed in half
		if(driveLeftStick.getTrigger())
		{
			right /= 2.0;
			left /= 2.0;
		}

		double avgStick = (right + left) / 2.0;

		if(!driveRightStick.getTrigger() && !shouldIHelpDriverDriveStraight())
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
			isGyroResetTelop = false;
		}
		else
		{
			if(isGyroResetTelop == false)
			{
				driveGyro.reset();
				isGyroResetTelop = true;
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
	 * Spin shooter wheels up to speed with door closed.
	 * Return true when wheels are at speed
	 */
	boolean spinShooterWheels(double frontWheel, double backWheel)
	{
		//Ensure we do not tell the shooter to spin backwards
		if(frontWheel < 0.0)
			frontWheel = 0.0;
		if(backWheel < 0.0)
			backWheel = 0.0;

		shooterWheelFront.setP(0.057);
		shooterWheelFront.setI(0.0001);
		shooterWheelFront.setD(1.3); //1.2

		shooterWheelBack.setP(0.057);
		shooterWheelBack.setI(0.0001);
		shooterWheelBack.setD(1.3);

		shooterWheelFront.set(-1.0 * frontWheel);
		shooterWheelBack.set(backWheel);

		avgShooterVelocityError = (shooterWheelFront.getClosedLoopError() + shooterWheelBack.getClosedLoopError()) / 2.0;

		if(avgShooterVelocityError < 200 && (shooterWheelBack.getSpeed() > (backWheel * 0.9))) //500
			return true;

		return false;
	}

	/*
	 * Stop shooter wheels from spinning
	 */
	void stopShooter()
	{
		shooterServo.set(servo_Down);

		shooterWheelFront.setP(0.0);
		shooterWheelFront.setI(0.0);
		shooterWheelFront.setD(0.0);
		shooterWheelBack.setP(0.0);
		shooterWheelBack.setI(0.0);
		shooterWheelBack.setD(0.0);

		shooterWheelFront.set(0.0);
		shooterWheelBack.set(0.0);

		agitatorMotor.set(1.0);
	}

	/*
	 * Perform task of shooting fuel
	 */
	void shootFuel(boolean useDistanceSensor, double frontVel, double backVel)
	{
		if(useDistanceSensor)
		{	//double shootRPM = calculateShotSpeedBasedOnDistance();

			if(frontVel != 0.0) //0.0 indicated error
			{
				if(spinShooterWheels(frontVel, backVel))
					shooterServo.set(servo_Up);
				else
					shooterServo.set(servo_Down);
			}
		}
		else
		{
			if(spinShooterWheels(frontVel, backVel)) //3400, 3500
				shooterServo.set(servo_Up);
			else
				shooterServo.set(servo_Down);
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
	 * Simple manual shooter control.  A more complex shooting scheme will be needed to step through opening the door and waiting until the shooter is at speed.
	 */
	void shootFuelControl()
	{
		double angleBoilerFoundDeg = 0.0;
		double sVel = 0.0;

		if(manipulatorStick.getY() > 0.1 || manipulatorStick.getY() < -0.1)
		{
			/*
			if(spinShooterWheels(manipulatorStick.GetY() * 3600.0, manipulatorStick.GetY() * 3400.0))
				shooterServo.Set(SERVO_UP);
			else
				shooterServo.Set(SERVO_DOWN);
			 */
		}
		else if(manipulatorStick.getRawButton(1) || manipulatorStick.getRawButton(2))
		{
			stoleDriveTrainControl = true;
			switch(shootFuelState)
			{
				case 0:
					resetDrive(use_Drive_Timer);
					shootFuelState++;
					break;

				case 1:
					//Search for boiler turning max 90 deg clockwise (assuming gear catcher is the front)
					if(turnGyro(-90.0, 0.3))
					{
						Timer.delay(0.25);
						resetDrive(use_Drive_Timer);
						shootFuelState = -1;
					}
					if(!photoElectricShooter.get())
					{
						stopRobotDrive();

						angleBoilerFoundDeg = driveGyro.getAngle();
						sVel = calculateShotSpeedBasedOnDistance();

						agitatorUp = false;
						agitatorTimer.reset();
						agitatorTimer.start();

						shootFuelState++;
					}
					break;

				case 2:
					//Attempt to keep the robot pointing in the correct direction
					if(!photoElectricShooter.get() && turnGyro(angleBoilerFoundDeg, 0.3))
					{
						if(manipulatorStick.getRawButton(2))
						{
							shootFuel(false, 3200.0, 3200.0); //Use the distance sensor to adjust shot speed set to true
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
		//Should set a dead-zone for this despite the speed controllers having one built in
		//gearCatcherScrew.Set(manipulatorStick.GetX());
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
			//leftDriveEncoder.Reset();
			rightDriveEncoder.reset();
			resetGyroAngle();
		}
	}

	/*
	 * Reset the Gyro position
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
		return (encTicks / drive_TicksPerRev) * 3.14 * 4.0;
	}

	/*
	 * Used during autonomous to turn the robot to a specified angle.
	 */
	boolean turnGyro(double rAngle, double maxTurnSpeed)
	{
		double error = 0.0;
		double speedToSet = 0.0;
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
				speedToSet = (error/270) + 0.2 + gyroKi; //140 0.2
				if(Math.abs(speedToSet) > maxTurnSpeed)
					speedToSet = maxTurnSpeed * (speedToSet < 0.0 ? -1.0 : 1.0);
				leftDriveMotor.set(speedToSet * batteryCompensationPct()); //0.8
				rightDriveMotor.set(speedToSet * batteryCompensationPct()); //0.8
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
				speedToSet = (error/270) - 0.2 - gyroKi;
				if(Math.abs(speedToSet) > maxTurnSpeed)
					speedToSet = maxTurnSpeed * (speedToSet < 0.0 ? -1.0 : 1.0);
				leftDriveMotor.set(speedToSet * batteryCompensationPct()); //-0.8
				rightDriveMotor.set(speedToSet * batteryCompensationPct()); //-0.8
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
	boolean turnGyro(double rAngle)
	{	double maxTurnSpeed = 0.5;
		double error = 0.0;
		double speedToSet = 0.0;
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
				speedToSet = (error/270) + 0.2 + gyroKi; //140 0.2
				if(Math.abs(speedToSet) > maxTurnSpeed)
					speedToSet = maxTurnSpeed * (speedToSet < 0.0 ? -1.0 : 1.0);
				leftDriveMotor.set(speedToSet * batteryCompensationPct()); //0.8
				rightDriveMotor.set(speedToSet * batteryCompensationPct()); //0.8
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
				speedToSet = (error/270) - 0.2 - gyroKi;
				if(Math.abs(speedToSet) > maxTurnSpeed)
					speedToSet = maxTurnSpeed * (speedToSet < 0.0 ? -1.0 : 1.0);
				leftDriveMotor.set(speedToSet * batteryCompensationPct()); //-0.8
				rightDriveMotor.set(speedToSet * batteryCompensationPct()); //-0.8
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
				//leftDriveMotor.Set(-velocityLeft);
				//rightDriveMotor.Set(velocityRight);
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
				//leftDriveMotor.Set(-velocityLeft);
				//rightDriveMotor.Set(velocityRight);
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
	 * Calculate the shooter speed based on ultrasonic measured distance
	 */
	double calculateShotSpeedBasedOnDistance()
	{
		double currentShooterDistanceInch = CalculateWallDistanceShooter(false) + 17.5;

		//Minimum shot distance
		if(currentShooterDistanceInch < 24.0)
			return 0.0;

		double shooterVelocity = 0.0;
		double shooterCalculatedRPM = 0.0;
		shooterVelocity = Math.sqrt(((currentShooterDistanceInch * 2.0) * gravity_in_S2) / Math.sin(2.0 * shooter_Angle_Degrees * pi / 180.0));
		shooterCalculatedRPM = (shooterVelocity * 60.0)/ (shooter_Wheel_Diameter_Inch * pi) * (shooter_PCT_Efficiency / 100.0);

		SmartDashboard.putNumber("Calculated Shot Speed (RPM): ", shooterCalculatedRPM);

		return shooterCalculatedRPM;
	}

	/*
	 * Used to calculate the robot distance from the wall
	 */
	double CalculateWallDistanceShooter(boolean averaged)
	{	averaged = true;
		double rawVoltage;
		double wallDistance;

		if(averaged)
			rawVoltage = (double)(wallDistanceSensorS.getAverageVoltage());
		else
			rawVoltage = (double)(wallDistanceSensorS.getVoltage());

		//MB1030
		double VFiveMM1 = 0.009671875;
		wallDistance = rawVoltage / VFiveMM1;

		return wallDistance;
	}

	/*
	 * Used to calculate the robot distance from the wall
	 */
	double CalculateWallDistanceR(boolean averaged)
	{
		averaged = true;
		double rawVoltage;
		double wallDistance;

		if(averaged)
			rawVoltage = (double)(wallDistanceSensorR.getAverageVoltage());
		else
			rawVoltage = (double)(wallDistanceSensorR.getVoltage());

		//MB1013
		double VFiveMM2 = 0.00488; //Old numbers 0.0048359375;  //((4.952 / 5120) * 5);
		wallDistance = ((rawVoltage * 5 * 0.0393) / VFiveMM2);  //Units inch

		//MB1030
		//double VFiveMM = 0.009671875;
		//wallDistance = rawVoltage / VFiveMM;

		return wallDistance;
	}

	/*
	 * Used to calculate the robot distance from the wall
	*/
	double CalculateWallDistanceL(boolean averaged)
	{	averaged = true;
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
		batteryScaleFactor = max_Battery / DriverStation.getInstance().getBatteryVoltage();

		return batteryScaleFactor;
	}

	/*
	 * Update the SmartDashboard
	 */
	void updateDashboard()
	{
		//SmartDashboard.putNumber("Wall Distance Right: ", CalculateWallDistanceR(false));
		SmartDashboard.putNumber("Wall Distance Right: ", shooterWheelBack.getSpeed());
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
	}

	/*
	 * Align robot parallel with wall
	 */
	double performRobotFaceAlignment()
	{
		double angleToTurnDegrees = 0.0;
		try
		{
			angleToTurnDegrees = (Math.tan((CalculateWallDistanceR(false) - CalculateWallDistanceL(false)) / 22.625) * 180.0) / pi;
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
	 * Get both middle and close hopper of balls (RED)
	 * I've assumed that negative angles will turn clockwise relative to the gear catcher being the front
	 * I've assume positive drive motor speeds will drive the robot in reverse (relative to the gear catcher being the front)
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
				resetDrive(use_Drive_Timer);
				autoState++;
				break;
			case 1:
				//Drive the robot in reverse to get to middle hopper
				if(autoDriveRobot(0.5, 0.5, 0, closeHopperDistance, use_Drive_Timer))
				{
					Timer.delay(0.25);
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 2:
				//Turn intake side towards hopper
				if(turnGyro(-90.0 * angleMultiplier))
				{
					//Timer.delay(0.25);
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 3:
				//Drive the robot reverse to trigger hopper
				if(autoDriveRobot(0.6, 0.6, 1.5, 84, use_Drive_Timer) || WaitAsyncUntil(2.0, true))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 4:
				//Wait some time for the hopper to empty into robot
				if(WaitAsyncUntil(1.0, true))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 5:
				//Drive the robot forward away from hopper
				if(autoDriveRobot(-0.5, -0.5, 0, 36, use_Drive_Timer))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 6:
				//Turn to start position for searching
				if(!isRED)
				{
					//BLUE
					if(turnGyro(-160.0))
					{
						Timer.delay(0.25);
						resetDrive(use_Drive_Timer);
						autoState++;
					}
				}
				else
				{
					//RED
					if(turnGyro(70.0)) //60
					{
						Timer.delay(0.25);
						resetDrive(use_Drive_Timer);
						autoState++;
					}
				}
				break;
			case 7:
				//Drive the robot forward away from hopper
				if(isRED)
				{
					if(autoDriveRobot(-0.5, -0.5, 0, 34, use_Drive_Timer))  //36
					{
						resetDrive(use_Drive_Timer);
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
					resetDrive(use_Drive_Timer);
					autoState = -1;
				}
				if(!photoElectricShooter.get())
				{
					stopRobotDrive();

					//sVel = calculateShotSpeedBasedOnDistance();

					//agitatorUp = false;
					//agitatorTimer.Reset();
					//agitatorTimer.Start();

					autoState++;
				}
				break;
			case 9:
				//Turn until not seeing boiler
				if(turnGyro(4.0, 0.3))
				{
					Timer.delay(0.25);
					resetDrive(use_Drive_Timer);

					stopRobotDrive();

					sVel = calculateShotSpeedBasedOnDistance();

					agitatorUp = false;
					agitatorTimer.reset();
					agitatorTimer.start();
					autoState++;
				}
				break;
			case 10:
				shootFuel(true, sVel, sVel);
				if(WaitAsyncUntil(7.0, true))
				{
					stopShooter();
					resetDrive(use_Drive_Timer);
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
		if(driveLeftStick.getRawButton(10) || driveLeftStick.getRawButton(7))
		{
			stoleDriveTrainControl2 = true;
			reverseNInches(3.0);
		}
		else
		{
			stoleDriveTrainControl2 = false;
			driveRevState = 0;
		}
	}

	void reverseNInches(double driveDistanceToReverse)
	{
		switch(driveRevState)
				{
					case 0:
						resetDrive(use_Drive_Timer);
						driveRevState++;
						break;
					case 1:
						//Drive the robot in reverse
						if(autoDriveRobot(0.25, 0.25, 0, driveDistanceToReverse, use_Drive_Timer))
						{
							resetDrive(use_Drive_Timer);
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
	 * I've assume positive drive motor speeds will drive the robot in reverse (relative to the gear catcher being the front)
	 */
	void score_GearPosition1_Autonomous(boolean isRED, boolean shootFuelAfterGear)
	{
		shootFuelAfterGear = true;
		double angleErrorFromUltrasonics = 0.0;
		double angleToTurn = -113.0; //For RED -120
		double angleForBoiler = 110.0; //90
		double distanceToDrive = 76.0; //For RED. was 72
		float distanceForGearPlacement = 36; //30
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
				resetDrive(use_Drive_Timer);
				autoState++;
				break;
			case 1:
				//Drive the robot in reverse
				if(autoDriveRobot(0.5, 0.5, 0, distanceToDrive, use_Drive_Timer))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
					//autoState= -1;
				}
				break;
			case 2:
				if(turnGyro(angleToTurn, 0.35))
				{
					Timer.delay(0.25);
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 3:
				//Home gear catcher
				gearCatcherState = 0;
				findGearCatcherLift();

				//Drive the robot forward to get a bit closer to airship
				if(autoDriveRobot(-0.4, -0.4, 1.3, 20, use_Drive_Timer))
				{
					resetDrive(use_Drive_Timer);
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
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 5:
				if(turnGyro(angleErrorFromUltrasonics))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 6:
				//Search for the peg using the photoelectric sensor
				if(findGearCatcherLift())
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				/*else if(WaitAsyncUntil(5.0, true))
				{
					autoState = -1; //Abort due to timeout
				}*/
				break;
			case 7:
				//Drive the robot forward to get the gear on the peg
				if(autoDriveRobot(-0.3, -0.3, 0.5, distanceForGearPlacement, use_Drive_Timer) || WaitAsyncUntil(2.0, true))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 8:
				//Wait some time for the pilot to remove the gear.  Would be better to have a sensor for this.
				if(WaitAsyncUntil(2.0, true))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 9:
				//Drive the robot reverse
				if(autoDriveRobot(0.4, 0.4, 0.6, 26, use_Drive_Timer))
				{
					resetDrive(use_Drive_Timer);

					if(shootFuelAfterGear)
						autoState++;
					else
						autoState = 6;
				}
				break;
			case 10:
				if(turnGyro(angleForBoiler))
				{
					Timer.delay(0.25);

					//agitatorUp = false;
					//agitatorTimer.Reset();
					//agitatorTimer.Start();

					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 11:
				if(turnGyro(-1.0 * angleForBoiler, 0.3))
				{
					Timer.delay(0.25);
					resetDrive(use_Drive_Timer);
					autoState = -1;
				}
				if(!photoElectricShooter.get())
				{
					stopRobotDrive();
					sVel = calculateShotSpeedBasedOnDistance();

					agitatorUp = false;
					agitatorTimer.reset();
					agitatorTimer.start();

					autoState++;
				}
				break;
			case 12:
				shootFuel(true, sVel, sVel);
				if(WaitAsyncUntil(5.0, true))
				{
					stopShooter();
					resetDrive(use_Drive_Timer);
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
	 * I've assume positive drive motor speeds will drive the robot in reverse (relative to the gear catcher being the front)
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
				resetDrive(use_Drive_Timer);
				autoState++;
				break;
			case 1:
				//Home gear catcher
				gearCatcherState = 0;
				findGearCatcherLift();

				//Drive the robot forward
				if(autoDriveRobot(-0.5, -0.5, 0, distanceToDrive, use_Drive_Timer))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
					//autoState= -1;
				}
				break;
			case 2:
				//Search for the peg using the photoelectric sensor
				if(findGearCatcherLift())
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				/*else if(WaitAsyncUntil(5.0, true))
				{
					autoState = -1; //Abort due to timeout
				}*/
				break;
			case 3:
				//Drive the robot forward to get the gear on the peg
				if(autoDriveRobot(-0.3, -0.3, 0.5, 36, use_Drive_Timer) || WaitAsyncUntil(1.5, true))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 4:
				//Wait some time for the pilot to remove the gear.  Would be better to have a sensor for this.
				if(WaitAsyncUntil(3.0, true))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 5:
				//Drive the robot reverse
				if(autoDriveRobot(0.4, 0.4, 0.6, 24, use_Drive_Timer))
				{
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;
			case 6:
				if(isRED)
				{
					if(turnGyro(-180.0))
					{
						resetDrive(use_Drive_Timer);
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
					resetDrive(use_Drive_Timer);

					stopRobotDrive();

					sVel = calculateShotSpeedBasedOnDistance();

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
				if(WaitAsyncUntil(5.0, true))
				{
					stopShooter();
					resetDrive(use_Drive_Timer);
					autoState++;
				}
				break;

			default:
				stopRobotDrive();
				break;
		}
}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		DriverStation.Alliance allianceColor;
		boolean isAllianceRED = false;
		System.out.println("Auto Selected:" + autoSelected);
		gearCatcherState = 0;
		autoState = 0;
		allianceColor = DriverStation.getInstance().getAlliance();
		if(allianceColor == DriverStation.Alliance.Red)
		{
			isAllianceRED = true;
		}
		updateDashboard();
		switch (autoSelected) {
		case autoNameGear1:
			score_GearPosition1_Autonomous(isAllianceRED, true); //set second param to false to retry gear
			break;
		case autoNameGear2:
			score_GearPosition2_Autonomous(isAllianceRED, true);
			break;
		case autoNameGear3:
			score_GearPosition1_Autonomous(!isAllianceRED, false);
			break;
		case autoNameTwoHopper:
			score_TwoHopper_Autonomous(isAllianceRED);
			break;
		case defaultAuto:
		default:
			doNothingAutonomous();
			break;
		}
		updateDashboard();
		Timer.delay(0.005);
	}
	/**
	 * This function is called periodically during operator control
	 */
	@Override
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

					calculateShotSpeedBasedOnDistance();
					// wait for a motor update time
		Timer.delay(0.005);}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	     LiveWindow.run();
	}

}
