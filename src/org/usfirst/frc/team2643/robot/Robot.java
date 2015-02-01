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
// DIVIDE ENCODERS BY 360 && MULTIPLY BY DIAMETER AND Math.PI//
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	Talon forwardLeftMotor = new Talon(5);
	Talon backLeftMotor = new Talon(6);
	Talon forwardRightMotor = new Talon(7);
	Talon backRightMotor = new Talon(9);
	SpeedController linearSlideMotor1;
	SpeedController linearSlideMotor2;
	SpeedController strafeMotor1;
	SpeedController strafeMotor2;
	DigitalInput slideSwitchTop;
	DigitalInput slideSwitchBottom;
	DigitalInput frontLeftLimitSwitch;
	DigitalInput frontRightLimitSwitch;
	Joystick padDriver = new Joystick(1);
	Joystick padOperator = new Joystick(2);
	Encoder frontRightEncoder = new Encoder( 2 , 3 );
	Encoder frontLeftEncoder = new Encoder ( 0 , 1 );
	Encoder strafeEncoder = new Encoder ( 6 , 7 );
	Encoder linearSlideEncoder = new Encoder ( 4 , 5 );
	int state = 0;
	int backDistance = 0;
	int forwardDistance = 0;
	int leftDistance = 0;
	int rightDistance = 0;
	int upDistance = 0;
	int downDistance = 0;
	int counter = 1;
	int autonDistance = 0;
	int forwardSpeed = 1;
	int backSpeed = -1;
	int slideSpeedUp = 1;
	int slideSpeedDown = -1;
	int leftSpeed = 1;
	int rightSpeed = 1;
	int stop = 0;
	int diameter = 0;
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

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
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

    public void autonomousInit() 
    {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
        frontRightEncoder.reset();
        frontLeftEncoder.reset();
        strafeEncoder.reset();
        linearSlideEncoder.reset();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() 
    {
    	if(linearSlideEncoder.get() <= 360 )
    	{
    		linearSlideMotor1.set( - linearSlideEncoder.get() );
    		linearSlideMotor2.set( - linearSlideEncoder.get() );
    	}
        Scheduler.getInstance().run();
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
        			if(frontLeftLimitSwitch.get() && frontRightLimitSwitch.get() )
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
    			if(frontLeftLimitSwitch.get() && frontRightLimitSwitch.get() )
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
        			if(frontLeftLimitSwitch.get() && frontRightLimitSwitch.get() )
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
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

	/**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit()
    {

    }
    
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
    //if number is to large, set to one or negative one
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
        Scheduler.getInstance().run();
        
        if(padDriver.getRawButton(1))
        {
        	double frontLeftSpeed = limit( (padDriver.getY() + padDriver.getZ()) / 2.0 );
        	double frontRightSpeed = limit( (padDriver.getY() - padDriver.getZ()) / 2.0 );
        	double backLeftSpeed = limit( (padDriver.getY() + padDriver.getZ()) / 2.0 ) ;
        	double backRightSpeed = limit( (padDriver.getY() - padDriver.getZ()) / 2.0 );
        	double strafeSpeed = padDriver.getX() / 2.0;
        	forwardLeftMotor.set( Math.sqrt(frontLeftSpeed) ); 
        	forwardRightMotor.set( Math.sqrt(frontRightSpeed) );
        	backLeftMotor.set( Math.sqrt(backLeftSpeed) ); 
        	backRightMotor.set( Math.sqrt(backRightSpeed) );
        	strafeMotor1.set( Math.sqrt(strafeSpeed) );
        	strafeMotor2.set( Math.sqrt(strafeSpeed) );
        	//moving at half the speed 
        }	
        else
        {
        	double frontLeftSpeed = limit( padDriver.getY() + padDriver.getZ() );
        	double frontRightSpeed = limit( padDriver.getY() - padDriver.getZ() );
        	double backLeftSpeed = limit(padDriver.getY() + padDriver.getZ() );
        	double backRightSpeed = limit( padDriver.getY() - padDriver.getZ() );
        	double strafeSpeed = padDriver.getX();
        	forwardLeftMotor.set( Math.sqrt(frontLeftSpeed) ); 
        	forwardRightMotor.set( Math.sqrt(frontRightSpeed) );
        	backLeftMotor.set( Math.sqrt(backLeftSpeed) ); 
        	backRightMotor.set( Math.sqrt(backRightSpeed) );     
        	strafeMotor1.set( Math.sqrt(strafeSpeed) );
        	strafeMotor2.set( Math.sqrt(strafeSpeed) );
        	//moving at full speed 
        }
        //Do Back Drive Soon
        if(padOperator.getRawButton(1))
        {
        	double linearSlideSpeed = limit(padOperator.getY() / 2.0);
        	linearSlideMotor1.set( linearSlideSpeed );
        	linearSlideMotor2.set( linearSlideSpeed );
        	//linear slide moves determined by the position of the Y axis on the operator pad 
        }
        else
        {
        	double linearSlideSpeed = limit(padOperator.getY());
        	linearSlideMotor1.set( linearSlideSpeed );
        	linearSlideMotor2.set( linearSlideSpeed );
        	//linear slide moves determined by the position of the Y axis on the operator pad 
        }
        
        if(padOperator.getRawButton(7) || padOperator.getRawButton(8))
        {
        	if(frontLeftLimitSwitch.get() && frontLeftLimitSwitch.get())
        	{
        		linearSlideMotor1.set(slideSpeedUp);
        		linearSlideMotor2.set(slideSpeedUp);
        	}
        }
        else
        {
        	linearSlideMotor1.set(stop);
        	linearSlideMotor2.set(stop);
        }
        
        if(padOperator.getRawButton(1))
        {
        	//set tote position 1
        }
        else if(padOperator.getRawButton(1) && padOperator.getRawButton(6))
        {
        	//set tote plus container position 1
        }
        
        if(padOperator.getRawButton(2))
        {
        	//set tote position 2
        }
        else if(padOperator.getRawButton(2) && padOperator.getRawButton(6))
        {
        	//set tote plus container position 2
        }
        
        if(padOperator.getRawButton(3))
        {
        	//set tote position 3
        }
        else if(padOperator.getRawButton(3) && padOperator.getRawButton(6))
        {
        	//set tote plus container position 3
        }
        
        if(padOperator.getRawButton(4))
        {
        	//set tote position 4
        }
        else if(padOperator.getRawButton(4) && padOperator.getRawButton(6))
        {
        	//set tote plus container position 4
        }
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
