
package org.usfirst.frc.team2643.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team2643.robot.commands.ExampleCommand;
import org.usfirst.frc.team2643.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	SpeedController linearSlideMotor1;
	SpeedController linearSlideMotor2;
	SpeedController forwardLeftMotor;
	SpeedController forwardRightMotor;
	SpeedController backLeftMotor;
	SpeedController backRightMotor;
	SpeedController strifeMotor;
	DigitalInput switchTop;
	DigitalInput switchBottom;
	DigitalInput frontLeftLimitSwitch;
	DigitalInput frontRightLimitSwitch;
	Joystick pad = new Joystick(1);
	Encoder frontRightEncoder = new Encoder( 1 , 2 );
	Encoder frontLeftEncoder = new Encoder ( 1 , 2 );
	Encoder strifeEncoder = new Encoder ( 1 , 2);
	Encoder linearSlideEncoder = new Encoder ( 1 , 2 );
	int state = 0;
	int backDistance = 0;
	int forwardDistance = 0;
	int leftDistance = 0;
	int rightDistance = 0;
	int upDistance = 0;
	int downDistance = 0;
	int counter = 1;
	int autonDistance = 0;
	int forward = 1;
	int back = -1;
	int up = 1;
	int down = -1;
	int left = 1;
	int right = 1;
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;
	
	
	
    Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
		oi = new OI();
		frontRightEncoder.reset();
		frontLeftEncoder.reset();
		strifeEncoder.reset();
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
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() 
    {
        Scheduler.getInstance().run();
        switch (state)
        {
        	case 0:
        		forwardLeftMotor.set(forward);
                forwardRightMotor.set(forward);
                backLeftMotor.set(forward);
                backRightMotor.set(forward);
                if(frontLeftLimitSwitch.get() && frontRightLimitSwitch.get() )
                {
                	counter ++;
                	state = 1;
                }
                break;
                //move forward and both limit switches are hit moves to state 1
                
        	case 1:
        		forwardLeftMotor.set(0);
                forwardRightMotor.set(0);
                backLeftMotor.set(0);
                backRightMotor.set(0);
                linearSlideMotor1.set(up);
                linearSlideMotor2.set(up);
                if(switchTop.get() || linearSlideEncoder.get() >= upDistance)
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
                //motor stop, slide moves up, if slide reaches top or goes a certain distance goes to state 2, or MOVES TO STATE 8
                
        	case 2:
        		linearSlideMotor1.set(0);
                linearSlideMotor2.set(0);
                forwardLeftMotor.set(back);
                forwardRightMotor.set(back);
                backLeftMotor.set(back);
                backRightMotor.set(back);
                if(frontLeftEncoder.get() >= backDistance || frontRightEncoder.get() >= backDistance)
                {
                	state = 3;
                }
        		break;
                //slide stops, moves back, if moves a certain distance, move to state 3
        		
        	case 3:
        		forwardLeftMotor.set(0);
                forwardRightMotor.set(0);
                backLeftMotor.set(0);
                backRightMotor.set(0);
                strifeMotor.set(left);
                if(strifeEncoder.get() >= leftDistance)
                {
                	state = 4;
                }
        		break;
                //motors stop, and move to right/left, if goes a certain distance, move to state 4
        		
        	case 4:
        		forwardLeftMotor.set(forward);
                forwardRightMotor.set(forward);
                backLeftMotor.set(forward);
                backRightMotor.set(forward);
                if(frontLeftEncoder.get() >= forwardDistance || frontRightEncoder.get() >= forwardDistance)
                {
                	state = 5;
                }
        		break;
        		//move forward, if the robot goes a certain distance, it moves to state 5
        		
        	case 5:
        		linearSlideMotor1.set(down);
        		linearSlideMotor2.set(down);
        		if(linearSlideEncoder.get() >= downDistance)
        		{
        			state = 6;
        		}
        		break;
        		//linear slide goes down, if slide is down a certain distance move to state 6
        		
        	case 6:
        		forwardLeftMotor.set(back);
                forwardRightMotor.set(back);
                backLeftMotor.set(back);
                backRightMotor.set(back);
                if(frontLeftEncoder.get() >= backDistance || frontRightEncoder.get() >= backDistance)
                {
                	state = 7;
                }
                break;
                //moves back, if goes a certain distance move to state 7
                
        	case 7:
        		linearSlideMotor1.set(down);
        		linearSlideMotor2.set(down);
        		if(linearSlideEncoder.get() >= downDistance)
        		{
        			state = 0;
        		}
        		break;
        		//slide moves down, if slide moves a certain distance goes back to state 0
        		
        	case 8:
        		//Do something to go to auton zone
                break;
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

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
        Scheduler.getInstance().run();
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() 
    {
    	//myEncoder.get(); Gets the distance moved ( 250 per revolution )
    	//myEncoder.reset() Resets encoder, do this at beginning of program
    	
    	
        LiveWindow.run();
        if(pad.getY() > 0)
        {
        	forwardLeftMotor.set(1);
        	forwardRightMotor.set(1);
        	backLeftMotor.set(1);
        	backRightMotor.set(1);
        	// all motors moves forward
        }
        else if(pad.getY() < 0)
        {
        	forwardLeftMotor.set(-1);
        	forwardRightMotor.set(-1);
        	backLeftMotor.set(-1);
        	backRightMotor.set(-1);
        	//all motors move back
        }
        else if(pad.getY() == 0)
        {
        	forwardLeftMotor.set(0);
        	forwardRightMotor.set(0);
        	backLeftMotor.set(0);
        	backRightMotor.set(0);
        	//stops all motors
        }
        	
        if(pad.getX() > 0)
        {
        	strifeMotor.set(1);
        	//either moving left or right
        }
        else if(pad.getX() < 0)
        {
        	strifeMotor.set(-1);
        	//either moving left or right
        }
        else if(pad.getX() == 0)
        {
        	strifeMotor.set(0);
        	//no longer moving left or right
        }
        
        if(pad.getRawButton(1))
        {
        	forwardLeftMotor.set(-1);
            forwardRightMotor.set(1);
            backLeftMotor.set(-1);
            backRightMotor.set(1);
            //turnRight
        }
        else if(pad.getRawButton(3))
        {
        	forwardLeftMotor.set(1);
            forwardRightMotor.set(-1);
            backLeftMotor.set(1);
            backRightMotor.set(-1);
            //turnLeft
        }
        else
        {
        	forwardLeftMotor.set(0);
            forwardRightMotor.set(0);
            backLeftMotor.set(0);
            backRightMotor.set(0);
            //stop
        }
        
        if(pad.getRawButton(4) && !switchTop.get())
        {
        	linearSlideMotor1.set(1);
        	linearSlideMotor2.set(1);
        	//linear slide moves up
        }
        else if(pad.getRawButton(2) && !switchBottom.get())
        {
        	linearSlideMotor2.set(-1);
        	linearSlideMotor1.set(-1);
        	//linear slide moves down
        }
        else
        {
        	linearSlideMotor2.set(0);
        	linearSlideMotor1.set(0);
        	//linear slide doesn't move
        }
     
        
    }
}
