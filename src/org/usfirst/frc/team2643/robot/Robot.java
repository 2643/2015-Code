//Strafe encoder left = negative right = positive
//Tote distance: 1.52 meters
package org.usfirst.frc.team2643.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.CameraServer;

import org.usfirst.frc.team2643.robot.commands.ExampleCommand;
import org.usfirst.frc.team2643.robot.subsystems.ExampleSubsystem;

public class Robot extends IterativeRobot 
{
	Timer autonomousTimer = new Timer();
	double LSAccelerationAmount = 0.01;
	double LSAccelerationTime = 0.05;
	Timer LSAccelerationTimer = new Timer();
	double LSCurrentSpeed = 0.0;
	double LSMaxSpeed = 0.6;
	/*
	CameraServer server = CameraServer.getInstance();
	
	//AxisCamera camera = new AxisCamera("10.26.43.10");
	public Robot(){
		server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("10.26.43.11:80");
	}
	*/
	double slowDriveFactor = 1.0;
	
	boolean gyroRegulateForward = false;
	boolean gyroRegulateStrafe = false;
	double gyroDesiredAngle;
	
	AnalogInput gyroscope = new AnalogInput( 0 );
	Gyro gyro = new Gyro( gyroscope );
	
	//Dash board
	SmartDashboard smartDashboard = new SmartDashboard();
	
	//Controllers
	Joystick driverController = new Joystick(0);
	
	//Drive motors
	Talon forwardLeftMotor = new Talon(5);
	Victor backLeftMotor = new Victor(6);
	Victor forwardRightMotor = new Victor(7);
	Talon backRightMotor = new Talon(9);
	
	Talon strafeMotor1 = new Talon(1);
	Talon strafeMotor2 = new Talon(2);
	
	//Linear slide
	Talon linearSlideMotor1 = new Talon(3);
	Talon linearSlideMotor2 = new Talon(4);
	
	//Digital Inputs
	DigitalInput slideSwitchTop = new DigitalInput(6);
	DigitalInput slideSwitchBottom = new DigitalInput(7);
	
	DigitalInput linearHookLimitSwitchLeft = new DigitalInput(10);
	DigitalInput linearHookLimitSwitchRight = new DigitalInput(11);
	
	//Encoders
	Encoder leftEncoder = new Encoder( 5, 4 );
	Encoder rightEncoder = new Encoder( 3 , 2 );
	Encoder strafeEncoder = new Encoder( 9 , 8 );
	Encoder linearSlideEncoder = new Encoder ( 1 , 0 );
	
	//Constants
	double slowLinearSpeedFactor = 2.0;
	int linearSlideAutoButton1 = 7;
	int linearSlideAutoButton2 = 8;
	int containerButton = 6;
	int totePosition1 = 1;
	int totePosition2 = 2;
	int totePosition3 = 3;
	int totePosition4 = 4;
	double dPadSpeed = 0.4;
	double encoderRotation = 360.0;
	int strafeButton1 = 5;
	int strafeButton2 = 6;
	
	//Linear slide
	
	int LSpresetState = 0;
	boolean zeroEncoder = true;
	
	//
	int backDistance = 0;
	int forwardDistance = 0;
	int leftDistance = 0;
	int rightDistance = 0;
	int upDistance = 0;
	int downDistance = 0;
	int counter = 1;
	int autonDistance = 0;
	
	//
	int forwardSpeed = 1;
	int backSpeed = -1;
	int leftSpeed = 1;
	int rightSpeed = 1;
	int stop = 0;
	double slideSpeedUp = 0.5;
	double slideSpeedDown = -0.5;
	double slowDriveSpeedFactor = 0.5;
	
	//
	double leftControlSpeed;
	double rightControlSpeed;
	double strafeControlSpeed;
	
	//Linear Slide variables
	double toteHeight = 670.0;
	double linearSlideGroundHeight = 880.0;
	int button = 0;
	double thumbStickTolerance = 0.1;
	
	
	//
	double diameter = 0.0;
	double desiredPreset;
	double desired;
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;
		
    Command autonomousCommand;
    
    int autonomousState = 0; //For state machine
    int startPosition = 0; //Starting position of the robot
    int toteNumber = 0; //Total number of bins that must be picked up
    int autonomousCycle = 0; //Number of bins that have currently been picked up
    
    double autonomousForwardSpeed = 0.3; //How fast the robot will move forward / backwards
    double autonomousStrafeSpeed = 0.6; //How fast the robot will strafe
    
    double currentAutoStrafeSpeed = 0.0;
    int toteDistance = 1250; //Distance between two bins
    int autonomousZoneDistance = 500; //How far forward robot must move from first staging zone to reach autonomous zone
    int autonomousZoneBackupDistance = 500; //How far to back away after bin(s) have been placed in autonomous zone
    int autonomousForwardDistance = 0; //How far forward robot must move when handling a new stack of bins
    int autonomousBackupDistance = 100; //How far back robot must move after placing a bin on top of another bin ( so it can lower the linear slide )
    
    int autonomousPlaceHeight2 = 800; //How low linear slide must be in order to let go of the top bin after stacking it
    int autonomousPlaceHeight1 = 200; //How low linear slide must be in order to drive forward and pick up the bottom bin
    
    int currentAutonomousDirection; //Direction robot is strafing
    int desiredAutonomousPosition; //Position robot is trying to strafe to ( relative to the robot's position at the beginning of autonomous )
    
    boolean autoStrafing = false;
    int autoWheelDifference;
    int autoWheelChange;
    double autoWheelChangeFactor;
    double autoRightSpeed;
    
    /*
     * Data for what directions / distances to drive
     * Position 1 is is the robot is placed on the leftmost bin, 2 for middle, and 3 for rightmost
     * Data is organized in pairs
     * First number is the direction ( 1 to drive to the right, -1 to drive to the left )
     * Second number is the distance divided by the distance between bins ( 2 means shift over by two bins )
     */
    
    int[][] autonomousDriveData =
    	{
    		{ 1 , 1 , 1 , 1 } , //Position 1
    		{ -1 , 1 , 1 , 2 } , //Position 2
    		{ -1 , 1 , -1 , 1 } //Position 3
    	};
    
    //runs at the beginning of the program
    public void robotInit()
    {
		oi = new OI();
        autonomousCommand = new ExampleCommand();
    }
    
    public void teleopInit()
    {
        if (autonomousCommand != null) autonomousCommand.cancel();
        
        zeroEncoder = true;
        System.out.println( "Teleop init" );
        LSAccelerationTimer.start();
        LSAccelerationTimer.reset();
    }
    
    public void teleopPeriodic()
    {
    	Scheduler.getInstance().run();
    	
    	masterLinearSlide();
    	masterDrive();
    }
    
	public void autonomousInit()
    {
		gyro.reset();
		desiredAutonomousPosition = 0;
		
		System.out.println( "Autonomous Init" );
		autonomousCycle = 0;
		
		autonomousState = 0;
		
    	rightEncoder.reset();
    	leftEncoder.reset();
    	strafeEncoder.reset();
    	
    	startPosition = 2;//(int) Math.round( SmartDashboard.getNumber( "DB/Slider 0" ) ) - 1;
    	toteNumber = 3;//(int) Math.round( SmartDashboard.getNumber( "DB/Slider 1" ) );
    	
    	System.out.println( "Start position:  " + startPosition + "     Tote number:  " + toteNumber );
    	
    	autonomousTimer.start();
    	linearSlideEncoder.reset();
    }
    
    public void autonomousPeriodic()
    {
    	/*
    	if( strafeEncoder.get() < 1000 )
    	{
    		controlDriving( -gyro.getAngle() * 0.065 , gyro.getAngle() * 0.065 , 0.6 ); //This is to make the robot strafe
    		autonomousTimer.reset();
    	}
    	else if( !( !linearHookLimitSwitchLeft.get() && !linearHookLimitSwitchRight.get()  ) && autonomousTimer.get() > 0.3 )
    	{
    		controlDriving( 0.4 , 0.4 + ( gyro.getAngle() * 0.03 ) , 0.0 ); //This is to make the robot drive forward
    	}
    	else
    	{
    		controlDriving( 0.0 , 0.0 , 0.0 );
    		if( linearSlideEncoder.get() < 1000 && autonomousTimer.get() > 0.3 )
    		{
    			linearSlideMotor1.set( 0.4 );
    			linearSlideMotor2.set( 0.4 );
    		}
    	}
    	*/
    		
    	smartAutonomous();
    	System.out.println( autonomousState );
    	//System.out.println( "|A|  " + gyro.getAngle() );
    	//controlDriving( -gyro.getAngle() * 0.065 , gyro.getAngle() * 0.065 , 0.6 ); strafe
    	//controlDriving( 0.6 , 0.6 + ( gyro.getAngle() * 0.03 ) , 0.0 ); Drive forward straight
    }
    
    public void masterLinearSlide()
    {
    	if( driverController.getRawButton( 8 ) )
    	{
    		zeroEncoder = true;
    	}
    	
    	if( zeroEncoder )
    	{
    		if( slideSwitchBottom.get() )
    		{
    			zeroEncoder = false;
    			linearSlideEncoder.reset();
    		}
    		else
    		{
    			linearSlideMotor1.set(slideSpeedDown * 0.5);
        		linearSlideMotor2.set( slideSpeedDown * 0.5 );
    		}
    	}
    	else
    	{
    		humanControlledLinearSlide();
    	}
    }
    
    public void humanControlledLinearSlide()
    {
    	boolean button1 = driverController.getRawButton(1);
    	boolean button2 = driverController.getRawButton(2);
    	boolean button3 = driverController.getRawButton(3) && !linearHookLimitSwitchLeft.get() && !linearHookLimitSwitchRight.get();
    	boolean button4 = driverController.getRawButton(4);
    	
    	double manualSpeed = driverController.getRawAxis(5);
    	int newButton = 0;
    	
    	/*
    	 * if button 1 - 4 is pressed
    	 * it'll change the newButton to the button
    	 * it then goes through a cycle to see weather if its > than or < than the desire presets
    	 */
    	if( button1 || button2 || button3 || button4 )
		{
			if(button4)
			{
				newButton = 3;
			}
			else if( button2 )
			{
				newButton = 2;
			}
			else if( button1 || button3 )
			{
				newButton = 1;
			}
			
			desiredPreset = ( newButton - 1 ) * toteHeight + linearSlideGroundHeight;
			if( Math.abs( linearSlideEncoder.get()- desiredPreset ) < 100.0 )
			{
				desiredPreset -= 170;
			}
			
			if( linearSlideEncoder.get() < desiredPreset)
			{
				LSpresetState = 2;
			}
			else if( linearSlideEncoder.get() > desiredPreset )
			{
				LSpresetState = 1;
				LSCurrentSpeed = 0.0;
			}
			else
			{
				LSpresetState = 0;
			}
		}
    	
    	if( manualSpeed <= -thumbStickTolerance || manualSpeed >= thumbStickTolerance )
    	{
    		LSpresetState = 0;
    	}
    	
    	switch( LSpresetState )
    	{
    		case 0:
    			stickDriveHookDrive();
    			break;
    		
    		case 1:
    			if( LSAccelerationTimer.get() >= LSAccelerationTime && LSCurrentSpeed > -LSMaxSpeed )
    			{
    				LSAccelerationTimer.reset();
    				LSCurrentSpeed -= LSAccelerationAmount;
    			}
    			
    			linearSlideMotor1.set( LSCurrentSpeed );
    			linearSlideMotor2.set( LSCurrentSpeed );
    		
    			if( linearSlideEncoder.get() <= desiredPreset || slideSwitchBottom.get() )
    			{
    				desired = linearSlideEncoder.get();
    				LSpresetState = 0;
    			}
    			break;    		
    		
    		case 2:	
    			linearSlideMotor1.set(slideSpeedUp);
    			linearSlideMotor2.set(slideSpeedUp);
    			
    			if( linearSlideEncoder.get() >= desiredPreset || slideSwitchTop.get() )
    			{
    				desired = linearSlideEncoder.get();
    				LSpresetState = 0;
    			}
    			break;
    	}
    }
    
    public void stickDriveHookDrive()
    {
    	double stickRightLinearSlideDrive = driverController.getRawAxis(5);// be sure it is Twist first
    	  	
    	if( stickRightLinearSlideDrive < 0 && !slideSwitchTop.get() )
    	{
    		linearSlideMotor1.set( -stickRightLinearSlideDrive * .6 );
        	linearSlideMotor2.set( -stickRightLinearSlideDrive * .6 );
    		
    	}
    	else if( stickRightLinearSlideDrive > 0 && !slideSwitchBottom.get() )
    	{
    		linearSlideMotor1.set( -stickRightLinearSlideDrive * .6 );
        	linearSlideMotor2.set( -stickRightLinearSlideDrive * .6 );
    	}
    	else
    	{
    		linearSlideMotor1.set(0);
    		linearSlideMotor2.set(0);
    	}
    }
    
    //this function adds all the drivers together into one super driver
    public void masterDrive()
    {
        int POV = driverController.getPOV();
        if( POV >= 0 )//check used to be 1//
        {
        	dPadDriver();
        }
        else 
        {
        	slowSpeedControl();
        	stickDrive();
        	strafeControl();
        }
        controlDriving( leftControlSpeed, rightControlSpeed, strafeControlSpeed );
    }
    
    public void controlDriving( double leftControlSpeed, double rightControlSpeed, double strafeControlSpeed )
    {     	
    	if( Math.abs( leftControlSpeed ) + Math.abs( rightControlSpeed ) < 0.1 && Math.abs( strafeControlSpeed ) > 0.1 )
    	{
    		if( !gyroRegulateStrafe )
    		{
    			gyroRegulateStrafe = true;
    			gyroDesiredAngle = gyro.getAngle();
    		}
    		double angleDifference = gyroDesiredAngle - gyro.getAngle();
    		leftControlSpeed = angleDifference * 0.065;
    		rightControlSpeed = -angleDifference * 0.065;
    		System.out.println( "Strafe straight" );
    	}
    	else
    	{
    		gyroRegulateStrafe = false;
    	}
    	
    	if( Math.abs( leftControlSpeed ) + Math.abs( rightControlSpeed ) > 0.1 && Math.abs( strafeControlSpeed ) < 0.1 )
    	{
    		System.out.println( "Drive straight" );
    		if( !gyroRegulateForward )
    		{
    			gyroRegulateForward = true;
    			gyroDesiredAngle = gyro.getAngle();
    		}
    		double angleDifference = gyroDesiredAngle - gyro.getAngle();
    		rightControlSpeed -= angleDifference * 0.03;
    	}
    	else
    	{
    		gyroRegulateForward = false;
    	}
    	
        backLeftMotor.set( leftControlSpeed ); 
        forwardLeftMotor.set( leftControlSpeed ); 
    	
    	forwardRightMotor.set( -rightControlSpeed );
        backRightMotor.set( -rightControlSpeed );
        
        strafeMotor1.set( -strafeControlSpeed );
        strafeMotor2.set( -strafeControlSpeed );
    }
    
    //this is the p-pad driver 
    public void dPadDriver()
    {
    	int POV = driverController.getPOV();
    	if( POV == 0 || POV == 45 || POV == 315 )
    	{
    		leftControlSpeed = dPadSpeed;
    		rightControlSpeed = dPadSpeed;
    	}
    	else if( POV == 135 || POV == 180 || POV == 225 )
    	{
    		leftControlSpeed = -dPadSpeed;
    		rightControlSpeed = -dPadSpeed;
    	}
    	else
    	{
    		leftControlSpeed = 0.0;
    		rightControlSpeed = 0.0;
    	}
    	
    	if( POV == 45 || POV == 90 || POV == 135 )
    	{
    		strafeControlSpeed = dPadSpeed;
    	}
    	else if( POV== 225 || POV == 270 || POV == 315 )
    	{
    		strafeControlSpeed = -dPadSpeed;
    	}
    	else
    	{
    		strafeControlSpeed = 0.0;
    	}
    }
    
    public void slowSpeedControl()
    {
    	if( driverController.getRawButton( 5 ) && driverController.getRawButton( 6 ) )
    	{
    		slowDriveFactor = slowDriveSpeedFactor * slowDriveSpeedFactor;
    	}
    	else if( driverController.getRawButton( 5 ) || driverController.getRawButton( 6 ) )
    	{
    		slowDriveFactor = slowDriveSpeedFactor;
    	}
    	else
    	{
    		slowDriveFactor = 1.0;
    	}
    }
    
  //this is the stick driver for the controller
    public void stickDrive()
    {
    	leftControlSpeed = limit( (-driverController.getY() + driverController.getX() ) * slowDriveFactor );
    	rightControlSpeed = limit( (-driverController.getY() - driverController.getX() ) * slowDriveFactor );
    }
    
    public void strafeControl()
    {
    	strafeControlSpeed = ( driverController.getRawAxis( 3 ) - driverController.getRawAxis( 2 ) ) * slowDriveFactor;
    }
    
    //Gets the average of the two drive encoders
    public int driveEncoderAverage()
    {
    	return ( leftEncoder.get() + rightEncoder.get() ) / 2;
    }
    
    //Makes sure the motor.set() value does not go past 1 or -1
    public double limit( double number )
    {
    	if(number > 1.0)
    	{
    		number = 1.0;
    	}
    	else if(number < -1.0)
    	{
    		number = -1.0;
    	}
		return number;
    }
    
    public void backDrive()
    {
       	if( linearSlideEncoder.get() <= desired )
    	{
    		linearSlideMotor1.set( limit( linearSlideEncoder.get() / encoderRotation ) );
    		linearSlideMotor2.set( limit( linearSlideEncoder.get() / encoderRotation ) );
    	}
    	else
    	{
    		linearSlideMotor1.set(0);
    		linearSlideMotor2.set(0);
    	}
    }
    
    //driving with the stick
    
    public void testPeriodic(){LiveWindow.run();}
    public void disabledPeriodic(){Scheduler.getInstance().run();}
    public void disabledInit() {}
    
    public void smartAutonomous()
    {
    	switch( autonomousState )
    	{
    		case 0: //Send linear slide down for the purpose of zeroing the encoder
    			
    			//Lower linear slide
    			linearSlideMotor1.set( slideSpeedDown );
    			linearSlideMotor2.set( slideSpeedDown );
    			
    			controlDriving( 0.0 , 0.0 , 0.0 );
    			
    			if( slideSwitchBottom.get() ) //When slide has reached the bottom
    			{
    				//Stop linear slide
    				linearSlideMotor1.set( 0.0 );
        			linearSlideMotor2.set( 0.0 );
        			
        			//Zero encoder
        			linearSlideEncoder.reset();
        			
        			/*
        			 * If there are any bins to be picked up, move forward
        			 * Otherwise drive to the side for the purpose of entering the autonomous zone
        			 */
        			if( autonomousCycle < toteNumber )
        			{
        				autonomousState = 1;
        			}
        			else
        			{
        				autoStrafing = true;
        				autonomousState = 4;
        			}
    			}
    			break;
    		
    		case 1: //Drive robot forward until bin is hit
    			controlDriving( autonomousForwardSpeed , autonomousForwardSpeed + ( gyro.getAngle() * 0.03 ) , 0.0 );
    			//If bin has been hit
    			if( !linearHookLimitSwitchLeft.get() && !linearHookLimitSwitchRight.get() )
    			{
    				//Raise linear slide
    				linearSlideMotor1.set( slideSpeedUp );
    				linearSlideMotor2.set( slideSpeedUp );
    				
    				autonomousForwardDistance = driveEncoderAverage();
    				
    				autonomousState = 2;
    			}
    			break;
    			
    		case 2: //Lift the bin
    			controlDriving( 0.0 , 0.0 , 0.0 );
    			
    			//If bin has been lifted high enough
    			if( linearSlideEncoder.get() >= linearSlideGroundHeight )
    			{
    				//Stop linear slide
    				linearSlideMotor1.set( 0.0 );
					linearSlideMotor2.set( 0.0 );
					
					autonomousState = 3;
    			}
    			break;
    		
    		case 3: //Back up
    			
    			controlDriving( -autonomousForwardSpeed , -autonomousForwardSpeed + ( gyro.getAngle() * 0.03 ) , 0.0 );
    			
    			//If robot has backed up far enough
    			if( driveEncoderAverage() <= -200 )
    			{
    				//Increment the autonomous cycle, which keeps track of how many bins have been picked up
    				autonomousCycle++;
    				
    				/*
    				 * If there are still bins to be picked up, grab the strafe direction and strafe distance to the next bin from autonomousDriveData
    				 * Otherwise calculate strafe direction and distance to autonomous zone
    				 */
    				if( autonomousCycle < toteNumber )
    				{
    					int[] data = autonomousDriveData[ startPosition ];
    					currentAutonomousDirection = data[ ( autonomousCycle - 1 ) * 2 ];
    					int distance = data[ ( ( autonomousCycle - 1 ) * 2 ) + 1 ];
    					desiredAutonomousPosition += ( currentAutonomousDirection * distance * toteDistance );
    				}
    				else
    				{
    					desiredAutonomousPosition = -( toteDistance * startPosition );
    					currentAutonomousDirection = -1;
    				}
    				
    				currentAutoStrafeSpeed = autonomousStrafeSpeed * ( double )( currentAutonomousDirection );
					autoStrafing = true;
    				
    				autonomousState = 4;
    			}
    			break;
    		
    		case 4: //Drive to the side
    			
    			controlDriving( -gyro.getAngle() * 0.065 , gyro.getAngle() * 0.065 , currentAutoStrafeSpeed );
    			
    			//If the robot has strafed far enough
    			if( ( strafeEncoder.get() <= desiredAutonomousPosition && currentAutonomousDirection < 0 ) ||
    				( strafeEncoder.get() >= desiredAutonomousPosition && currentAutonomousDirection > 0 ) )
    			{
    				//Drive forward
    				autoStrafing = false;
    				
    				/*
    				 * If there are still bins to be picked up, set robot on path to next bin
    				 * Otherwise set robot on path to autonomous zone
    				 */
    				if( autonomousCycle < toteNumber )
    				{
        				autonomousState = 5;
    				}
    				else
    				{
        				autonomousState = 9;
    				}
    			}
    			break;
    			
    		case 5: //Drive forward to place bin on top of other bin
    			controlDriving( autonomousForwardSpeed , autonomousForwardSpeed + ( gyro.getAngle() * 0.03 ) , 0.0 );
    			
    			//If robot has moved far enough forward
    			if( driveEncoderAverage() >= autonomousForwardDistance - 100 )
    			{
    				//Lower linear slide
    				linearSlideMotor1.set( slideSpeedDown );
    				linearSlideMotor2.set( slideSpeedDown );
    				
    				autonomousState = 6;
    			}
    			break;
    			
    		case 6: //Let go of top bin
    			controlDriving( 0.0 , 0.0 , 0.0 );
    			
    			//If slide has been lowered far enough to release the second bin
    			if( linearSlideEncoder.get() <= autonomousPlaceHeight2 )
    			{
    				//Stop linear slide
    				linearSlideMotor1.set( 0.0 );
    				linearSlideMotor2.set( 0.0 );
    				
    				autonomousState = 7;
    			}
    			break;
    		
    		case 7: //Back up
    			controlDriving( -autonomousForwardSpeed , -autonomousForwardSpeed + ( gyro.getAngle() * 0.03 ) , 0.0 );
    			
    			//If robot has moved back far enough
    			if( driveEncoderAverage() <= autonomousForwardDistance - autonomousBackupDistance )
    			{
    				//Lower linear slide
    				linearSlideMotor1.set( slideSpeedDown );
    				linearSlideMotor2.set( slideSpeedDown );
    				
    				autonomousState = 8;
    			}
    			break;
    		
    		case 8: //Lower linear slide enough to pick up bottom bin
    			controlDriving( 0.0 , 0.0 , 0.0 );
    			
    			//If slide has been lowered enough to pick up the bottom bin
    			if( slideSwitchBottom.get() )
    			{
    				//Stop linear slide
    				linearSlideMotor1.set( 0.0 );
    				linearSlideMotor2.set( 0.0 );
    				
    				autonomousState = 1;
    			}
    			break;
    		
    		case 9: //Drive forward until autonomous zone is reached
    			controlDriving( autonomousForwardSpeed , autonomousForwardSpeed + ( gyro.getAngle() * 0.03 ) , 0.0 );
    			
    			//When autonomous zone has been reached
    			if( driveEncoderAverage() >= autonomousZoneDistance )
    			{
    				//Lower linear slide
    				linearSlideMotor1.set( slideSpeedDown );
    				linearSlideMotor2.set( slideSpeedDown );
    				
    				autonomousState = 10;
    			}
    			break;
    		
    		case 10: //Lower linear slide to bottom
    			controlDriving( 0.0 , 0.0 , 0.0 );
    			
    			//When slide has reached bottom
    			if( slideSwitchBottom.get() )
    			{
    				//Stop linear slide
    				linearSlideMotor1.set( 0.0 );
    				linearSlideMotor2.set( 0.0 );
    				
    				autonomousState = 11;
    			}
    			break;
    			
    		case 11: //Back up robot then stop it
    			
    			//When robot has backed up far enough
    			if( driveEncoderAverage() <= autonomousZoneBackupDistance )
    			{
    				//Stop robot
    				controlDriving( 0.0 , 0.0 , 0.0 );
    			}
    			else
    			{
    				controlDriving( -autonomousForwardSpeed , -autonomousForwardSpeed + ( gyro.getAngle() * 0.03 ) , 0.0 );
    			}
    			break;
    	}
    }
}