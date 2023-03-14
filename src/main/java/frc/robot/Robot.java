// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler; 



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
//Initializes on Startup
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() 
  {
    CameraServer.startAutomaticCapture();
    // Instantiate our RobotContainer.  This will perform all our button bindings
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() 
  {
    // Runs the Scheduler
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() 
  {
    m_robotContainer.autonomousInitialization();
  }

  @Override
  public void autonomousPeriodic() 
  {
    m_robotContainer.autonomousDrive();
  }

  @Override
  public void teleopInit() 
  {
    // This stops the autonomous when the teleop begins
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() 
  {
    //Controller Input
    m_robotContainer.controllerInput();

    //Drive Train Logic And Power
    m_robotContainer.driveTrainLogic();
    
    //Display Acceleration Values
    m_robotContainer.acceleration();
    
    //Display Motor Voltage Values
    m_robotContainer.motors();
  }

  @Override
  public void disabledInit() 
  {
    m_robotContainer.motorsDisabled();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() 
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
