// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Defining DriveTrain Hardware using VictorSPX motor controllers.
  VictorSPX leftMotorFront = new VictorSPX(Constants.LEFT_MOTOR_A * -1);
  VictorSPX leftMotorBack = new VictorSPX(Constants.LEFT_MOTOR_B * -1);
  VictorSPX rightMotorFront = new VictorSPX(Constants.RIGHT_MOTOR_A);
  VictorSPX rightMotorBack = new VictorSPX(Constants.RIGHT_MOTOR_B);

  double stickInput;
  double right;
  double left;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void controllerInput() {
    XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER);

    stickInput = driverController.getRawAxis(Constants.CONTROLLER_RX_AXIS);
    right = driverController.getRawAxis(Constants.CONTROLLER_RIGHT_TRIGGER)*1.2;
    left = driverController.getRawAxis(Constants.CONTROLLER_LEFT_TRIGGER)*-1.2;
  }

  public void driveTrainLogic() {
    //Drivetrain Logic
    double forward = right + left;
    double driveLeft = forward - stickInput;
    double driveRight = forward + stickInput;

    //Sending Power to Motors
    leftMotorFront.set(ControlMode.PercentOutput, driveLeft*0.7);
    leftMotorBack.set(ControlMode.PercentOutput, driveLeft*0.7);
    rightMotorFront.set(ControlMode.PercentOutput, driveRight*0.7);
    rightMotorBack.set(ControlMode.PercentOutput, driveRight*0.7);
  }

  public void motorsDisabled() {
    leftMotorFront.set(ControlMode.PercentOutput, 0);
    leftMotorBack.set(ControlMode.PercentOutput, 0);
    rightMotorFront.set(ControlMode.PercentOutput, 0);
    rightMotorBack.set(ControlMode.PercentOutput, 0);
  }
}
