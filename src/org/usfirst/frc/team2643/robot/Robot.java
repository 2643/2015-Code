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
	DigitalInput slideSwitchTop;
	DigitalInput slideSwitchBottom;
	
	DigitalInput linearHookLimitSwitch;
		
	//Controllers
	Joystick driverController = new Joystick(1);
	Joystick operatorController = new Joystick(2);
	
	//Encoders
	Encoder frontRightEncoder = new Encoder( 2 , 3 );
	Encoder frontLeftEncoder = new Encoder ( 0 , 1 );
	Encoder strafeEncoder = new Encoder ( 6 , 7 );
	Encoder linearSlideEncoder = new Encoder ( 4 , 5 );
	
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
	
	//
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
		
    Command autonomousCommand;
    
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
    public void autonomousInit() 
    {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
        
        frontRightEncoder.reset();
        frontLeftEncoder.reset();
        strafeEncoder.reset();
        linearSlideEncoder.reset();
    }

    
    public void autonomousPeriodic() 
    {
        Scheduler.getInstance().run();
        backDrive();
        // if(/*position left*/)
        {
        	switch (state)
        	{
        		case 0: //move forward and both limit switches are hit moves to state 1
        			
        			if(slideSwitchBottom.get())
        			{
        				linearSlideMotor1.set(stop);
        				linearSlideMotor2.set(stop);
        			}
        			else
        			{
        				linearSlideMotor1.set(slideSpeedDown);
        				linearSlideMotor2.set(slideSpeedDown);
        			}
        			
        			forwardLeftMotor.set(forwardSpeed);
        			forwardRightMotor.set(forwardSpeed);
        			backLeftMotor.set(forwardSpeed);
        			backRightMotor.set(forwardSpeed);
        			if(linearHookLimitSwitch.get())
        			{
        				counter ++;
        				state = 1;
        			}
        			break;
                
        		case 1://motor stop, slide moves up, if slide reaches top or goes a certain distance goes to state 2, or MOVES TO STATE 8
        			forwardLeftMotor.set(stop);
        			forwardRightMotor.set(stop);
        			backLeftMotor.set(stop);
        			backRightMotor.set(stop);
        			linearSlideMotor1.set(slideSpeedUp);
        			linearSlideMotor2.set(slideSpeedUp);
        			if(slideSwitchTop.get() || linearSlideEncoder.get() >= upDistance)//maybe change the distance
        			{
        				if(counter >= 3)
        				{
        					state = 8;
        				}
        				else
        				{
        					state = 2;
        				}
        			}
        			break;
                
        		case 2://slide stops, robot moves back, if moves a certain distance, move to state 3
        			linearSlideMotor1.set(stop);
        			linearSlideMotor2.set(stop);
        			forwardLeftMotor.set(backSpeed);
        			forwardRightMotor.set(backSpeed);
        			backLeftMotor.set(backSpeed);
        			backRightMotor.set(backSpeed);
        			if((frontLeftEncoder.get() / 360) * diameter * Math.PI >= backDistance || frontRightEncoder.get() >= backDistance)
        			{
        				state = 3;
        			}
        			break;
                
        		case 3://motors stop, and move to right/left, if goes a certain distance, move to state 4
        			forwardLeftMotor.set(stop);
        			forwardRightMotor.set(stop);
        			backLeftMotor.set(stop);
        			backRightMotor.set(stop);
        			strafeMotor1.set(rightSpeed);
        			strafeMotor2.set(rightSpeed);
                if(strafeEncoder.get() >= rightDistance)
                {
                	state = 4;
                }
        		break;

        		case 4://move forward, if the robot goes a certain distance, it moves to state 5
        			forwardLeftMotor.set(forwardSpeed);
        			forwardRightMotor.set(forwardSpeed);
        			backLeftMotor.set(forwardSpeed);
        			backRightMotor.set(forwardSpeed);
        			if(frontLeftEncoder.get() >= forwardDistance || frontRightEncoder.get() >= forwardDistance)
	                {
	                	state = 5;	
	                }
	        		break;
	        		
        		case 5://linear slide goes down, if slide is down a certain distance move to state 6
        			linearSlideMotor1.set(slideSpeedDown);
        			linearSlideMotor2.set(slideSpeedDown);
        			if(linearSlideEncoder.get() >= downDistance)
        			{
        				state = 6;
        			}
        			break;
        			
        		case 6://moves back, if goes a certain distance move to state 7
	        		forwardLeftMotor.set(backSpeed);
	                forwardRightMotor.set(backSpeed);
	                backLeftMotor.set(backSpeed);
	                backRightMotor.set(backSpeed);
	                if(frontLeftEncoder.get() >= backDistance || frontRightEncoder.get() >= backDistance)
	                {
	                	state = 7;
	                }
	                break;
                
        		case 7://slide moves down, if slide moves a certain distance goes back to state 0
        			linearSlideMotor1.set(slideSpeedDown);
	        		linearSlideMotor2.set(slideSpeedDown);
	        		if(linearSlideEncoder.get() >= downDistance)
	        		{
	        			state = 0;
	        		}
	        		break;
        		
        		case 8:
        			//Do something to go to auton zone
        			break;
        	}
        }
      //  else if(/*position middle*/)
        {
        	switch(state)
        	{
        	case 0://slide moves down and the wheels move forward, if goes a certain distance move to state 1
        		
        		if(slideSwitchBottom.get())
    			{
    				linearSlideMotor1.set(stop);
    				linearSlideMotor2.set(stop);
    			}
    			else
    			{
    				linearSlideMotor1.set(slideSpeedDown);
    				linearSlideMotor2.set(slideSpeedDown);
    			}
    			forwardLeftMotor.set(forwardSpeed);
    			forwardRightMotor.set(forwardSpeed);
    			backLeftMotor.set(forwardSpeed);
    			backRightMotor.set(forwardSpeed);
    			if(linearHookLimitSwitch.get())
    			{
    				counter ++;
    				state = 1;
    			}
    			break;
    			
    		case 1://motor stop, slide moves up, if slide reaches top or goes a certain distance goes to state 2, or MOVES TO STATE 8
    			forwardLeftMotor.set(stop);
    			forwardRightMotor.set(stop);
    			backLeftMotor.set(stop);
    			backRightMotor.set(stop);
    			linearSlideMotor1.set(slideSpeedUp);
    			linearSlideMotor2.set(slideSpeedUp);
    			if(slideSwitchTop.get() || linearSlideEncoder.get() >= upDistance)
    			{
    				if(counter >= 3)
    				{
    					state = 8;
    				}
    				else
    				{
    					state = 2;
    				}
    			}
    			break;
    			
    		case 2://slide stops, robot moves back, if moves a certain distance, move to state 3
    			linearSlideMotor1.set(stop);
    			linearSlideMotor2.set(stop);
    			forwardLeftMotor.set(backSpeed);
    			forwardRightMotor.set(backSpeed);
    			backLeftMotor.set(backSpeed);
    			backRightMotor.set(backSpeed);
    			if(frontLeftEncoder.get() >= backDistance || frontRightEncoder.get() >= backDistance)
    			{
    				state = 3;
    			}
    			break;
            
    		case 3://motors stop, and move to right/left, if goes a certain distance, move to state 4
    			forwardLeftMotor.set(stop);
    			forwardRightMotor.set(stop);
    			backLeftMotor.set(stop);
    			backRightMotor.set(stop);
    			strafeMotor1.set(leftSpeed);
    			strafeMotor2.set(leftSpeed);
    			if(strafeEncoder.get() >= leftDistance)
    			{
    				strafeMotor1.set(stop);
    				strafeMotor2.set(stop);
    				state = 4;
    			}
    			break;

    		case 4://move forward, if the robot goes a certain distance, it moves to state 5
    			forwardLeftMotor.set(forwardSpeed);
    			forwardRightMotor.set(forwardSpeed);
    			backLeftMotor.set(forwardSpeed);
    			backRightMotor.set(forwardSpeed);
    			if(frontLeftEncoder.get() >= forwardDistance || frontRightEncoder.get() >= forwardDistance)
                {
                	state = 5;	
                }
        		break;
        		
    		case 5://linear slide goes down, if slide is down a certain distance move to state 6
    			linearSlideMotor1.set(slideSpeedDown);
    			linearSlideMotor2.set(slideSpeedDown);
    			if(linearSlideEncoder.get() >= downDistance)
    			{
    				state = 6;
    			}
    			break;
    			
    		case 6://moves back, if goes a certain distance move to state 7
        		forwardLeftMotor.set(backSpeed);
                forwardRightMotor.set(backSpeed);
                backLeftMotor.set(backSpeed);
                backRightMotor.set(backSpeed);
                if(frontLeftEncoder.get() >= backDistance || frontRightEncoder.get() >= backDistance)
                {
                	state = 7;
                }
                break;
            
    		case 7://slide moves down, if slide moves a certain distance goes back to state 0
    			linearSlideMotor1.set(slideSpeedDown);
        		linearSlideMotor2.set(slideSpeedDown);
        		if(linearSlideEncoder.get() >= downDistance)
        		{
        			state = 8;
        		}
        		break;
        		
    		case 8:
    			linearSlideMotor1.set(stop);
    			linearSlideMotor2.set(stop);
    			strafeMotor1.set(rightDistance * 2 );
    			strafeMotor2.set(rightDistance * 2);
    			if(strafeEncoder.get() >= rightDistance)
    			{
    				state = 0;
    			}
    		
    		case 9:
    			//Do something to go to auton zone
    			break;
        	}
        }
      //  else if(/*right start*/)
        {
        	switch (state)
        	{
        		case 0: //move forward and both limit switches are hit moves to state 1
        			
        			if(slideSwitchBottom.get())
        			{
        				linearSlideMotor1.set(stop);
        				linearSlideMotor2.set(stop);
        			}
        			else
        			{
        				linearSlideMotor1.set(slideSpeedDown);
        				linearSlideMotor2.set(slideSpeedDown);
        			}
        			
        			forwardLeftMotor.set(forwardSpeed);
        			forwardRightMotor.set(forwardSpeed);
        			backLeftMotor.set(forwardSpeed);
        			backRightMotor.set(forwardSpeed);
        			if( linearHookLimitSwitch.get() )
        			{
        				counter ++;
        				state = 1;
        			}
        			break;
                
        		case 1://motor stop, slide moves up, if slide reaches top or goes a certain distance goes to state 2, or MOVES TO STATE 8
        			forwardLeftMotor.set(stop);
        			forwardRightMotor.set(stop);
        			backLeftMotor.set(stop);
        			backRightMotor.set(stop);
        			linearSlideMotor1.set(slideSpeedUp);
        			linearSlideMotor2.set(slideSpeedUp);
        			if(slideSwitchTop.get() || linearSlideEncoder.get() >= upDistance)
        			{
        				if(counter >= 3)
        				{
        					state = 8;
        				}
        				else
        				{
        					state = 2;
        				}
        			}
        			break;
                
        		case 2://slide stops, robot moves back, if moves a certain distance, move to state 3
        			linearSlideMotor1.set(stop);
        			linearSlideMotor2.set(stop);
        			forwardLeftMotor.set(backSpeed);
        			forwardRightMotor.set(backSpeed);
        			backLeftMotor.set(backSpeed);
        			backRightMotor.set(backSpeed);
        			if(frontLeftEncoder.get() >= backDistance || frontRightEncoder.get() >= backDistance)
        			{
        				state = 3;
        			}
        			break;
                
        		case 3://motors stop, and move to right/left, if goes a certain distance, move to state 4
        			forwardLeftMotor.set(stop);
        			forwardRightMotor.set(stop);
        			backLeftMotor.set(stop);
        			backRightMotor.set(stop);
        			strafeMotor1.set(rightSpeed);
        			strafeMotor2.set(rightSpeed);
                if(strafeEncoder.get() >= rightDistance)
                {
                	state = 4;
                }
        		break;

        		case 4://move forward, if the robot goes a certain distance, it moves to state 5
        			forwardLeftMotor.set(forwardSpeed);
        			forwardRightMotor.set(forwardSpeed);
        			backLeftMotor.set(forwardSpeed);
        			backRightMotor.set(forwardSpeed);
        			if(frontLeftEncoder.get() >= forwardDistance || frontRightEncoder.get() >= forwardDistance)
	                {
	                	state = 5;	
	                }
	        		break;
	        		
        		case 5://linear slide goes down, if slide is down a certain distance move to state 6
        			linearSlideMotor1.set(slideSpeedDown);
        			linearSlideMotor2.set(slideSpeedDown);
        			if(linearSlideEncoder.get() >= downDistance)
        			{
        				state = 6;
        			}
        			break;
        			
        		case 6://moves back, if goes a certain distance move to state 7
	        		forwardLeftMotor.set(backSpeed);
	                forwardRightMotor.set(backSpeed);
	                backLeftMotor.set(backSpeed);
	                backRightMotor.set(backSpeed);
	                if(frontLeftEncoder.get() >= backDistance || frontRightEncoder.get() >= backDistance)
	                {
	                	state = 7;
	                }
	                break;
                
        		case 7://slide moves down, if slide moves a certain distance goes back to state 0
        			linearSlideMotor1.set(slideSpeedDown);
	        		linearSlideMotor2.set(slideSpeedDown);
	        		if(linearSlideEncoder.get() >= downDistance)
	        		{
	        			state = 0;
	        		}
	        		break;
        		
        		
        		case 8:
        			//Do something to go to auton zone
        			break;
        	}
        }
    }
    
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
    
    //this is the driving code that will be used
    public void controlDriving( double leftControlSpeed, double rightControlSpeed, double strafeControlSpeed )
    {
        backLeftMotor.set( leftControlSpeed ); 
        forwardLeftMotor.set( leftControlSpeed ); 
    	
    	forwardRightMotor.set( -rightControlSpeed );
        backRightMotor.set( -rightControlSpeed );
        
        strafeMotor1.set( strafeControlSpeed );
        strafeMotor2.set( strafeControlSpeed );
    }
    
    //this is the p-pad driver 
    public void dPadDriver()
    {
    	int POV = driverController.getPOV();
    	if( POV == 1 || POV == 2 || POV == 8 )
    	{
    		leftControlSpeed = dPadSpeed;
    		rightControlSpeed = dPadSpeed;
    	}
    	else if( POV == 4 || POV == 5 || POV == 6 )
    	{
    		leftControlSpeed = -dPadSpeed;
    		rightControlSpeed = -dPadSpeed;
    	}
    	else
    	{
    		leftControlSpeed = 0.0;
    		rightControlSpeed = 0.0;
    	}
    	
    	if( POV == 2 || POV == 3 || POV == 4 )
    	{
    		strafeControlSpeed = dPadSpeed;
    	}
    	else if( POV== 6 || POV == 7 || POV == 8 )
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
    		leftControlSpeed = limit( (driverController.getY() + driverController.getZ()) / slowDriveSpeedFactor );
    		rightControlSpeed = limit( (driverController.getY() - driverController.getZ()) / slowDriveSpeedFactor );
    		strafeControlSpeed = driverController.getX() / slowDriveSpeedFactor;
    	}	
    	else
    	{
    		leftControlSpeed = limit( driverController.getY() + driverController.getZ() );
    		rightControlSpeed = limit( driverController.getY() - driverController.getZ() );
    		strafeControlSpeed = driverController.getX();	
    	}
    }
    
    //this function adds all the drivers together into one super driver
    public void masterDrive()
    {
        int POV = driverController.getPOV();
        if( POV >= 1 )
        {
        	dPadDriver();
        }
        else 
        {
        	stickDrive();
        }
        controlDriving( leftControlSpeed, rightControlSpeed, strafeControlSpeed );
    }
    
    //this is the point of view using hook drive 
    public void dPadHookDrive()
    {
    	int POV = operatorController.getPOV();
    	if( POV == 1)
    	{
    		linearSlideMotor1.set(slideSpeedUp);
    		linearSlideMotor2.set(slideSpeedUp);    		
    	}
    	else
    	{
    		linearSlideMotor1.set(slideSpeedDown);
    		linearSlideMotor2.set(slideSpeedDown);   
    	}
    }
    
    //driving with the stick
    public void stickDriveHookDrive()
    {
    	double operator = operatorController.getY();
    	linearSlideMotor1.set( operator );
    	linearSlideMotor2.set( operator );
    }
    
    //this controls all the drive code that the robot can do, such as d-pad and stick drive
    public void masterManualLinearDrive()
    {
    	int POV = operatorController.getPOV();
    	double operator = operatorController.getY();
    	boolean button5 = operatorController.getRawButton(5);
    	boolean button6 = operatorController.getRawButton(6);
    	
    	if( (button5 || button6) && linearHookLimitSwitch.get() )
    	{
    		linearSlideMotor1.set(slideSpeedUp);
    		linearSlideMotor2.set(slideSpeedUp);
    		desired = linearSlideEncoder.get();
    	}
    	else if( POV == 1 || POV == 5 )
    	{
    		dPadHookDrive();
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
    	boolean button1 = operatorController.getRawButton(1);
    	boolean button2 = operatorController.getRawButton(2);
    	boolean button3 = operatorController.getRawButton(3);
    	boolean button4 = operatorController.getRawButton(4);
    	int POV = operatorController.getPOV();
    	double operator = operatorController.getY();
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
        
        masterDrive();
        masterLinearSlide();
        backDrive();
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
