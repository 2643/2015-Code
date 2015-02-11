package org.usfirst.frc.team2643.robot;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team2643.robot.commands.ExampleCommand;
import org.usfirst.frc.team2643.robot.subsystems.ExampleSubsystem;

public class Robot extends IterativeRobot 
{
	
	//Drive motors
	Talon forwardLeftMotor = new Talon(5);
	Talon backLeftMotor = new Talon(6);
	Talon forwardRightMotor = new Talon(7);
	Talon backRightMotor = new Talon(9);
	
	SpeedController strafeMotor1;
	SpeedController strafeMotor2;
	
	//Linear slide
	SpeedController linearSlideMotor1;
	SpeedController linearSlideMotor2;
	
	//Digital Inputs
	DigitalInput slideSwitchTop = new DigitalInput(6);
	DigitalInput slideSwitchBottom = new DigitalInput(7);
	
	DigitalInput linearHookLimitSwitch;
	DigitalInput frontSensor = new DigitalInput(9);
		
	//Controllers
	Joystick driverController = new Joystick(0);
	
	//Encoders
	Encoder frontRightEncoder = new Encoder( 4 , 5 );
	Encoder frontLeftEncoder = new Encoder ( 2 , 3 );
	Encoder strafeEncoder = new Encoder ( 6 , 7 );
	Encoder linearSlideEncoder = new Encoder ( 0 , 1 );
	
	//magic numbers
	int slowButton = 1;
	double slowDriveSpeedFactor = 2.0;
	double slowLinearSpeedFactor = 2.0;
	int linearSlideAutoButton1 = 7;
	int linearSlideAutoButton2 = 8;
	int containerButton = 6;
	int totePosition1 = 1;
	int totePosition2 = 2;
	int totePosition3 = 3;
	int totePosition4 = 4;
	double dPadSpeed = 0.5;
	double encoderRotation = 360 ;
	
	//states
	int state = 0;
	
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
	
	//
	double leftControlSpeed;
	double rightControlSpeed;
	double strafeControlSpeed;
	
	//Linear Slide variables
	double toteHeight = 0;
	double linearSlideGroundHeight = 0;
	int button = 0;
	double thumbStickTolerance = 0.1;
	
	
	//
	double diameter = 0.0;
	double desiredPreset;
	double desired;
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;
    int autonomousState = 0; //For state machine
    int startPosition = 0; //Starting position of the robot
    int toteNumber; //Total number of bins that must be picked up
    int autonomousCycle = 0; //Number of bins that have currently been picked up
    
    double autonomousForwardSpeed = 0.25; //How fast the robot will move forward / backwards
    double autonomousStrafeSpeed = 0.5; //How fast the robot will strafe
    
    int toteDistance = 0; //Distance between two bins
    int autonomousZoneDistance = 0; //How far forward robot must move from first staging zone to reach autonomous zone
    int autonomousZoneBackupDistance = 0; //How far to back away after bin(s) have been placed in autonomous zone
    int autonomousForwardDistance = 0; //How far forward robot must move when handling a new stack of bins
    int autonomousBackupDistance = 0; //How far back robot must move after placing a bin on top of another bin ( so it can lower the linear slide )
    
    int autonomousToteLiftHeight = 0; //How high bin must be lifted in order to be stacked on top of another bin
    int autonomousPlaceHeight2 = 0; //How low linear slide must be in order to let go of the top bin after stacking it
    int autonomousPlaceHeight1 = 0; //How low linear slide must be in order to drive forward and pick up the bottom bin
    
    int currentAutonomousDirection; //Direction robot is strafing
    int desiredAutonomousPosition; //Position robot is trying to strafe to ( relative to the robot's position at the beginning of autonomous )
    
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
    
    public void autonomousInit()
    {
    	frontRightEncoder.reset();
    	frontLeftEncoder.reset();
    	strafeEncoder.reset();
    	
    	//Set toteNumber
    	//Set start position
    }
    
    public void autonomousPeriodic()
    {
    	switch( autonomousState )
    	{
    		case 0: //Send linear slide down for the purpose of zeroing the encoder
    			
    			//Lower linear slide
    			linearSlideMotor1.set( slideSpeedDown );
    			linearSlideMotor2.set( slideSpeedDown );
    			
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
        				//Move forward
        				controlDriving( autonomousForwardSpeed , autonomousForwardSpeed , 0.0 );
        				
        				autonomousState = 1;
        			}
        			else
        			{
        				autonomousState = 4;
        			}
    			}
    			break;
    		
    		case 1: //Drive robot forward until bin is hit
    			
    			//If bin has been hit
    			if( linearHookLimitSwitch.get() )
    			{
    					//Stop robot
    					controlDriving( 0.0 , 0.0 , 0.0 );
    					
    					//Raise linear slide
    					linearSlideMotor1.set( slideSpeedUp );
    					linearSlideMotor2.set( slideSpeedUp );
    					
    					autonomousState = 2;
    			}
    			break;
    			
    		case 2: //Lift the bin
    			
    			//If bin has been lifted high enough
    			if( linearSlideEncoder.get() >= autonomousToteLiftHeight )
    			{
    				//Stop linear slide
    				linearSlideMotor1.set( 0.0 );
					linearSlideMotor2.set( 0.0 );
					
					//Drive backwards
					controlDriving( -autonomousForwardSpeed , -autonomousForwardSpeed , 0.0 );
					
					autonomousState = 3;
    			}
    			break;
    		
    		case 3: //Back up
    			
    			//If robot has backed up far enough
    			if( driveEncoderAverage() <= 0 )
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
    				
    				double speed = autonomousStrafeSpeed * ( double )( currentAutonomousDirection );
					controlDriving( 0.0 , 0.0 , autonomousStrafeSpeed * speed );
    				
    				autonomousState = 4;
    			}
    			break;
    		
    		case 4: //Drive to the side
    			
    			//If the robot has strafed far enough
    			if( ( strafeEncoder.get() <= desiredAutonomousPosition && currentAutonomousDirection < 0 ) ||
    				( strafeEncoder.get() >= desiredAutonomousPosition && currentAutonomousDirection > 0 ) )
    			{
    				//Drive forward
    				controlDriving( autonomousForwardSpeed , autonomousForwardSpeed , 0.0 );
    				
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
    			
    			//If robot has moved far enough forward
    			if( driveEncoderAverage() >= autonomousForwardDistance )
    			{
    				//Lower linear slide
    				linearSlideMotor1.set( slideSpeedDown );
    				linearSlideMotor2.set( slideSpeedDown );
    				
    				//Stop robot
    				controlDriving( 0.0 , 0.0 , 0.0 );
    				
    				autonomousState = 6;
    			}
    			break;
    			
    		case 6: //Let go of top bin
    			
    			//If slide has been lowered far enough to release the second bin
    			if( linearSlideEncoder.get() <= autonomousPlaceHeight2 )
    			{
    				//Stop linear slide
    				linearSlideMotor1.set( 0.0 );
    				linearSlideMotor2.set( 0.0 );
    				
    				//Move robot backwards
    				controlDriving( -autonomousForwardSpeed , -autonomousForwardSpeed , 0.0 );
    				
    				autonomousState = 7;
    			}
    			break;
    		
    		case 7: //Back up
    			
    			//If robot has moved back far enough
    			if( driveEncoderAverage() <= autonomousBackupDistance )
    			{
    				//Lower linear slide
    				linearSlideMotor1.set( slideSpeedDown );
    				linearSlideMotor2.set( slideSpeedDown );
    				
    				//Stop robot
    				controlDriving( 0.0 , 0.0 , 0.0 );
    				
    				autonomousState = 8;
    			}
    			break;
    		
    		case 8: //Lower linear slide enough to pick up bottom bin
    			
    			//If slide has been lowered enough to pick up the bottom bin
    			if( linearSlideEncoder.get() <= autonomousPlaceHeight1 )
    			{
    				//Stop linear slide
    				linearSlideMotor1.set( 0.0 );
    				linearSlideMotor2.set( 0.0 );
    				
    				//Move robot forward
    				controlDriving( autonomousForwardSpeed , autonomousForwardSpeed , 0.0 );
    				
    				autonomousState = 1;
    			}
    			break;
    		
    		case 9: //Drive forward until autonomous zone is reached
    			
    			//When autonomous zone has been reached
    			if( driveEncoderAverage() >= autonomousZoneDistance )
    			{
    				//Lower linear slide
    				linearSlideMotor1.set( slideSpeedDown );
    				linearSlideMotor2.set( slideSpeedDown );
    				
    				//Stop robot
    				controlDriving( 0.0 , 0.0 , 0.0 );
    				
    				autonomousState = 10;
    			}
    			break;
    		
    		case 10: //Lower linear slide to bottom
    			
    			//When slide has reached bottom
    			if( slideSwitchBottom.get() )
    			{
    				//Stop linear slide
    				linearSlideMotor1.set( 0.0 );
    				linearSlideMotor2.set( 0.0 );
    				
    				//Move robot backwards
    				controlDriving( -autonomousForwardSpeed , -autonomousForwardSpeed , 0.0 );
    				
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
    			break;
    	}
    }
    
    Command autonomousCommand;
    
    public void restartButtonToTheBottom()
    {
    	if( driverController.getRawButton(8) )
    	{
    		linearSlideMotor1.set(-1);
    		linearSlideMotor2.set(-1);
    		if( slideSwitchBottom.get() )
    		{
    			linearSlideEncoder.reset();
    		}
    	}	
    }
    
    //Gets the average of the two drive encoders
    public int driveEncoderAverage()
    {
    	return ( frontLeftEncoder.get() - frontRightEncoder.get() ) / 2;
    }
    
    /*
     * Controls the drive train
     * Accepts values for left motors, right motors, and strafe motors
     */
    public void controlDriving( double leftControlSpeed, double rightControlSpeed, double strafeControlSpeed )
    {
        backLeftMotor.set( leftControlSpeed );
        forwardLeftMotor.set( leftControlSpeed );
    	
    	forwardRightMotor.set( -rightControlSpeed );
        backRightMotor.set( -rightControlSpeed );
        
        strafeMotor1.set( strafeControlSpeed );
        strafeMotor2.set( strafeControlSpeed );
    }
    
    public void sensor()
    {
    	if(driverController.getPOV() == 0 && frontSensor.get() )
    	{
    		forwardLeftMotor.set(0);
    		backLeftMotor.set(0);
    		forwardRightMotor.set(0);
    		backRightMotor.set(0);
    	}
    }
    
    public int desire( int desiredPosition )
    {
    	if(linearSlideEncoder.get() <= 360 )
    	{
    		linearSlideMotor1.set( - linearSlideEncoder.get() );
    		linearSlideMotor2.set( - linearSlideEncoder.get() );
    	}
		return desiredPosition;
    }
    
    //runs at the beginning of the program
    public void robotInit() 
    {
		oi = new OI();
		frontRightEncoder.reset();
		frontLeftEncoder.reset();
		strafeEncoder.reset();
		linearSlideEncoder.reset();
		
        // instantiate the command used for the autonomous period
        autonomousCommand = new ExampleCommand();
    }
	
	public void disabledPeriodic() 
	{
		Scheduler.getInstance().run();
	}
	
	//Beginning of Autonomous Mode
    public void teleopInit() 
    {
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

	/**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit()
    {

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
    
    //might not be mine codes
    public void encoderControl(int position)
    {
    	int height = 100; //placeholder value from bottom to first tote position
    	if(linearSlideEncoder.get() < position * height){
    		linearSlideMotor1.set(0.5);
    		linearSlideMotor2.set(0.5);
    	}
    	if(linearSlideEncoder.get() > position * height){
    		linearSlideMotor1.set(0.5);
    		linearSlideMotor2.set(0.5);
    	}
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
    
    //this is the stick driver for the controller 
    public void stickDrive()
    {
    	if(driverController.getRawButton(slowButton))
    	{
    		leftControlSpeed = limit( (-driverController.getY() + driverController.getZ()) / slowDriveSpeedFactor );
    		rightControlSpeed = limit( (-driverController.getY() - driverController.getZ()) / slowDriveSpeedFactor );
    		strafeControlSpeed = driverController.getX() / slowDriveSpeedFactor;
    	}	
    	else
    	{
    		leftControlSpeed = limit( -driverController.getY() + driverController.getZ() );
    		rightControlSpeed = limit( -driverController.getY() - driverController.getZ() );
    		strafeControlSpeed = driverController.getX();	
    	}
    }
    
    //this function adds all the drivers together into one super driver
    public void masterDrive()
    {
        int POV = driverController.getPOV();
        if( POV >= 0 )
        {
        	dPadDriver();
        }
        else 
        {
        	stickDrive();
        }
        controlDriving( leftControlSpeed, rightControlSpeed, strafeControlSpeed );
    }
    
    //driving with the stick // fix
    public void stickDriveHookDrive()
    {
    	double operator = driverController.getThrottle();
	  	
    	if( operator < 0 && !slideSwitchTop.get() )
    	{
    		linearSlideMotor1.set( -operator );
        	linearSlideMotor2.set( -operator );
    		
    	}
    	else if( operator > 0 && !slideSwitchBottom.get() )
    	{
    		linearSlideMotor1.set( -operator );
        	linearSlideMotor2.set( -operator );
    	}
    	else
    	{
    		linearSlideMotor1.set(0);
    		linearSlideMotor2.set(0);
    	}
    }
    
    //this controls all the drive code that the robot can do, such as d-pad and stick drive
    public void masterManualLinearDrive()
    {    	
    	int POV = driverController.getPOV();
    	double operator = driverController.getY();
    	boolean button5 = driverController.getRawButton(5);
    	boolean button6 = driverController.getRawButton(6);
    	
    	if( (button5 || button6) && linearHookLimitSwitch.get() )
    	{
    		linearSlideMotor1.set(slideSpeedUp);
    		linearSlideMotor2.set(slideSpeedUp);
    		desired = linearSlideEncoder.get();
    	}
    	else if( POV == 0 || POV == 180 )
    	{
    		
    		desired = linearSlideEncoder.get();
    	}
    	else if( operator <= -thumbStickTolerance || operator >= thumbStickTolerance )
    	{
    		stickDriveHookDrive();
    		desired = linearSlideEncoder.get();
    	}
    	else
    	{
    		backDrive();
    	}
    }
     
    //this controls the master slide and goes through a switch go thorough the switch statement
    public void masterLinearSlide()
    {
    	boolean button1 = driverController.getRawButton(1);
    	boolean button2 = driverController.getRawButton(2);
    	boolean button3 = driverController.getRawButton(3);
    	boolean button4 = driverController.getRawButton(4);
    	int POV = driverController.getPOV();
    	double operator = driverController.getY();
    	int newButton;
    	int state = 0;
    	
    	/*
    	 * if button 1 - 4 is pressed
    	 * it'll change the newButton to the button
    	 * it then goes through a cycle to see weather if its > than or < than the desire presets
    	 */
    	if( button1 || button2 || button3 || button4 )
		{
			if(button4)
			{
				newButton = 4;
			}
			else if(button3)
			{
				newButton = 3;
			}
			else if(button2)
			{
				newButton = 2;
			}
			else
			{
				newButton = 1;
			}
			
			if( newButton != button )
	    	{
				button = newButton;
				desiredPreset = button * toteHeight - linearSlideGroundHeight;
				if( linearSlideEncoder.get() < desiredPreset) 
				{
					state = 2;
				}
				else if( linearSlideEncoder.get() > desiredPreset )
				{
					state = 1;
				}
				else
				{
					state = 0;
				}
				//update states
	    	}
		}
    	
    	if( POV == 1 || POV == 5 || operator <= -thumbStickTolerance || operator >= thumbStickTolerance )
    	{
    		state = 0;
    	}
    	
    	if( state != 0 )
    	{
    		desired = linearSlideEncoder.get();
    	}
    	
    	switch(state)
    	{
    	
    	case 0:
    		masterManualLinearDrive();
    		break;
    		
    	case 1:
    		
    		linearSlideMotor1.set(slideSpeedDown);
    		linearSlideMotor2.set(slideSpeedDown);
    		
    		if( linearSlideEncoder.get() <= desiredPreset )
    		{
    			desired = linearSlideEncoder.get();
    			state = 0;
    		}
    		break;    		
    		
    	case 2:
    		
    		linearSlideMotor1.set(slideSpeedUp);
    		linearSlideMotor2.set(slideSpeedUp);
    		    		
    		if( linearSlideEncoder.get() >= desiredPreset )
    		{
    			desired = linearSlideEncoder.get();
    			state = 0;
    		}
    		break;
    	}
    }
    
    //this functions is for back driving 
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
    
    //Operator and driving starts and moving motors
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run();
        
        stickDriveHookDrive();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() 
    {
    	//myEncoder.get(); Gets the distance moved ( 250 per revolution )
    	//myEncoder.reset() Resets encoder, do this at beginning of program
    	
    	
        LiveWindow.run();

    }
    
}
