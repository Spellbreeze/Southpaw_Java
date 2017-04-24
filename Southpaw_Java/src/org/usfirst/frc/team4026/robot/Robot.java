package org.usfirst.frc.team4026.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
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
    CANTalon gearCatcherScrew;
    CANTalon ballIntakeRoller1;
    CANTalon ballIntakeRoller2;
    Talon agitatorMotor;
   	Servo agitatorServo;
    Servo shooterServo;
  	AnalogInput wallDistanceSensorR;
   	AnalogInput wallDistanceSensorL;
   	DigitalInput gearCatcherLimitLeft;
   	DigitalInput gearCatcherLimitRight;
   	DigitalInput photoElectric;
   	DigitalInput photoElectricShooter;
   	AnalogGyro driveGyro;
    Joystick driveStick;
    Joystick manipulatorStick;
    Timer timer;
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	Timer autoDriveTimer;
	Timer agitatorTimer;
	Timer genericTimer;
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
		leftDriveMotor = new VictorSP(1);
		rightDriveMotor = new VictorSP(2);
		
		shooterWheelFront = new CANTalon(1);
		shooterWheelBack = new CANTalon(5);
		gearCatcherScrew = new CANTalon(3);
		ballIntakeRoller1 = new CANTalon(2);
		ballIntakeRoller2 = new CANTalon(4);
		
		agitatorServo = new Servo(5);
		shooterServo = new Servo(4);
		
		agitatorMotor = new Talon(2);
		
		wallDistanceSensorR = new AnalogInput(2);
		wallDistanceSensorL = new AnalogInput(1);
		
		gearCatcherLimitLeft = new DigitalInput(1);
		gearCatcherLimitLeft = new DigitalInput(0);
		photoElectric = new DigitalInput(2);
		photoElectricShooter = new DigitalInput(3);
		
		driveGyro = new AnalogGyro(0);
		
		myRobot = new RobotDrive(leftDriveMotor, rightDriveMotor);
       
	//	leftStick = new Joystick(1);
        driveStick = new Joystick(0);
        manipulatorStick = new Joystick(2);
        
        timer = new Timer();
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

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		tankDrive();
		if (manipulatorStick.getRawButton(1)){
			waitForSpeed(-3200.0,3400.0);
			shootFuel();
		}
		climb();
		scroll();
		if (manipulatorStick.getRawButton(4)){
			findGearCatcherLift();
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	     LiveWindow.run();
	}
	public void waitForSpeed(double desiredFrontSpeed, double desiredBackSpeed){
		int margin = 10;
		double targetFrontSpeed = desiredFrontSpeed;
	    double targetBackSpeed = desiredBackSpeed;
			shooterWheelFront.setF(0.0416);
			shooterWheelFront.setP(0.016);//0.086 0.096 0.046
			shooterWheelFront.setI(0.0);
			shooterWheelFront.setD(0.7);

		    shooterWheelBack.setF(0.0416);
			shooterWheelBack.setP(0.016);//0.086 0.096 0.046
			shooterWheelBack.setI(0.0);
			shooterWheelBack.setD(0.7);

		    // Start the motors
		    shooterWheelFront.set(targetFrontSpeed);
		    shooterWheelBack.set(targetBackSpeed);
		    
		
		double actualFrontSpeed = shooterWheelFront.getSpeed();
	    double actualBackSpeed = shooterWheelBack.getSpeed();
	    double actualFrontRPM = actualFrontSpeed * 6.8;
	    double actualBackRPM = actualBackSpeed * 6.8;
	    if (Math.abs(actualFrontRPM - targetFrontSpeed) < margin &&
	           Math.abs(actualBackRPM - targetBackSpeed) < margin) {
	            exitvalue = true;}
	    if (!manipulatorStick.getRawButton(1));{
	    	shooterWheelFront.setF(0.0);
	        shooterWheelBack.setF(0.0);
	    	exitvalue = false;
	    }
	}
	public void shootFuel(){
		double servo_up = .2;
		double servo_down = 1;
		if (manipulatorStick.getRawButton(1) && exitvalue == true){
			shooterServo.set(servo_up);
	}
		else {
			shooterServo.set(servo_down);
		}
	}
	public void climb(){
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
	public void tankDrive(){
		double right = driveStick.getZ();
		double left = driveStick.getY();
		leftDriveMotor.set(-left);
		rightDriveMotor.set(right);
	}
	public void scroll(){
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
	}
	public void driveStraight(){
		if (driveStick.getTrigger()){
			double targetAngle = driveGyro.getAngle();
			// Get the left joystick value
			double joystickRaw = driveStick.getZ();
	    	double currentAngle = driveGyro.getAngle();
	    
	    	// Could smooth or reverse
	    	double desiredVelocity = joystickRaw;

	    	double error = targetAngle - currentAngle;
	    	
	    	// The abs(error) should aways be less than or equal to 180 degrees
	    	while (error > 180.0) {
	    		error = error - 360.0;
	    	}
	    	while (error < -180) {
	    		error = error + 360.0;
	    	}
	    

	    	double correctionFactor = (error/75.0);
	    	
	    	double leftDriveVel;
			double rightDriveVel;
	    
	    	// If the error is positive, slow down the left drive

	    	// FIXME: Make sure the sign is right on all these
			leftDriveVel = -1 * (desiredVelocity - correctionFactor);
	    	rightDriveVel = desiredVelocity + correctionFactor;

	    	leftDriveMotor.set(leftDriveVel);
	    	rightDriveMotor.set(rightDriveVel);}
	}
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
}

