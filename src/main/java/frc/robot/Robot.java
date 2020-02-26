/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	public static final int LEDNUMBER = 90;
	private static final int MAX_MOVEMENT = 3000;
	LEDController led = new LEDController(LEDNUMBER);
	Encoder clawEncoder = new Encoder(0,1);
	
	Talon armChainTop = new Talon(3);
	Talon armChainBot = new Talon(7);
	Joystick control = new Joystick(0);
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		LedStrip allLEDs = new LedStrip(LEDNUMBER,1.0f);
		allLEDs.allOff();
		allLEDs.update();
		led.initialize();
		led.reset();
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
		p("Buttn Count: " + control.getButtonCount());
		led.bars();
	}

	public static void p(Object o)
	{
		System.out.println(o);
	}
	
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		led.strip.allOff();
		int dist = Math.max(1, Math.min(40, this.clawEncoder.get()/42));
		p(dist);
		led.strip.fill(255, 0, 0, 1, dist);
		led.strip.fill(255, 0, 0, 1, 90-dist);
		double dy = this.control.getRawAxis(5);
		if ((dy < 0 && clawEncoder.get() > MAX_MOVEMENT) || true) {
			p("Controller Y axis:" + control.getRawAxis(5));
			p("Encoder: " + clawEncoder.get());
			armChainBot.set(-dy);
			armChainTop.set(-dy);
		}
		led.strip.update();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

