// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler; 

// The VM runs this class automatically and calls the functions corresponding to each mode.
// If this is renamed/refactored update it in the build.gradle file.
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    //Controller Input
    m_robotContainer.controllerInput();

    //Drive Train logic and power
    m_robotContainer.driveTrainLogic();
  }



  // This function is called before teleopPeriodic().
  @Override
  public void teleopInit() 
  {
    // This stops the autonomous when the teleop begins
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }



  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {
    m_robotContainer.motorsDisabled();
  }



  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() 
  {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }



  @Override
  public void robotInit() 
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings
    m_robotContainer = new RobotContainer();
  }



  // The function is called every 20 ms
  // This runs after mode specific functions, but before LiveWindow and Smart Dashboard
  @Override
  public void robotPeriodic() 
  {
    // Runs the Scheduler
    CommandScheduler.getInstance().run();
  }



  @Override
  public void testInit() 
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }



  @Override
  public void disabledPeriodic() {}



  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}



  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}



  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}


  
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
