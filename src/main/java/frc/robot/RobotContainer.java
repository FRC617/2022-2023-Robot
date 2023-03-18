package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotContainer {
    VictorSPX leftMotorFront = new VictorSPX(Constants.LEFT_MOTOR_A);
    VictorSPX leftMotorBack = new VictorSPX(Constants.LEFT_MOTOR_B);
    VictorSPX rightMotorFront = new VictorSPX(Constants.RIGHT_MOTOR_A);
    VictorSPX rightMotorBack = new VictorSPX(Constants.RIGHT_MOTOR_B);

    CANSparkMax armMotorA = new CANSparkMax(Constants.ARM_MOTOR_A, MotorType.kBrushed);
    CANSparkMax armMotorB = new CANSparkMax(Constants.ARM_MOTOR_B, MotorType.kBrushed);

    double stickInput;
    double right;
    double left;

    double prevXAccel = 0;
    double prevYAccel = 0;

    boolean buttonA;
    boolean buttonB;
    boolean buttonX;
    boolean buttonY;
    boolean buttonADeterminer = true;

    boolean shoulderLeft;
    boolean shoulderRight;

    int buttonLevel = 0;
    
    Accelerometer accelerometer = new BuiltInAccelerometer();

    double autoStart = 0;
    boolean goForAuto = false;

    //Gets Controller Inputs
    public void controllerInput() {
        XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER);
    
        stickInput = driverController.getRawAxis(Constants.CONTROLLER_RX_AXIS);
        right = driverController.getRawAxis(Constants.CONTROLLER_RIGHT_TRIGGER)*1.2;
        left = driverController.getRawAxis(Constants.CONTROLLER_LEFT_TRIGGER)*-1.2;

        shoulderLeft = driverController.getRawButton(Constants.CONTROLLER_SHOULDER_LEFT);
        shoulderRight = driverController.getRawButton(Constants.CONTROLLER_SHOULDER_RIGHT);

        buttonA = driverController.getRawButton(Constants.CONTROLLER_BUTTON_A);
        buttonB = driverController.getRawButton(Constants.CONTROLLER_BUTTON_B);
        buttonX = driverController.getRawButton(Constants.CONTROLLER_BUTTON_X);
        buttonY = driverController.getRawButton(Constants.CONTROLLER_BUTTON_Y);
    }


    public void driveTrainLogic() {

        leftMotorFront.setInverted(true);
        leftMotorBack.setInverted(true);
        rightMotorFront.setInverted(false);
        rightMotorBack.setInverted(false);
        
        //Drivetrain Logic
        double forward = right + left;
        double driveLeft = forward - stickInput;
        double driveRight = forward + stickInput;


        //Count button levels up
        if (buttonA == true && buttonADeterminer == true) {
            buttonLevel += 1;
            buttonADeterminer = false;
        }

        if (buttonA == false) {
            buttonADeterminer = true;
        }

        //Reset button level to 0 after 4 presses.
        if (buttonLevel >= 2) {
            buttonLevel = 0;
        }
    
        //Sending Power to Motors at Different Strengths
        switch (buttonLevel) {
            case 0 :
                leftMotorFront.set(ControlMode.PercentOutput, driveLeft*0.7);
                leftMotorBack.set(ControlMode.PercentOutput, driveLeft*0.7);
                rightMotorFront.set(ControlMode.PercentOutput, driveRight*0.7);
                rightMotorBack.set(ControlMode.PercentOutput, driveRight*0.7);
                break;
            case 1 :
                leftMotorFront.set(ControlMode.PercentOutput, driveLeft*0.3);
                leftMotorBack.set(ControlMode.PercentOutput, driveLeft*0.3);
                rightMotorFront.set(ControlMode.PercentOutput, driveRight*0.3);
                rightMotorBack.set(ControlMode.PercentOutput, driveRight*0.3);
                break;
            case 2 :
                leftMotorFront.set(ControlMode.PercentOutput, driveLeft*0.3);
                leftMotorBack.set(ControlMode.PercentOutput, driveLeft*0.3);
                rightMotorFront.set(ControlMode.PercentOutput, driveRight*0.3);
                rightMotorBack.set(ControlMode.PercentOutput, driveRight*0.3);
                break;
            case 3 :
                leftMotorFront.set(ControlMode.PercentOutput, driveLeft*1);
                leftMotorBack.set(ControlMode.PercentOutput, driveLeft*1);
                rightMotorFront.set(ControlMode.PercentOutput, driveRight*1);
                rightMotorBack.set(ControlMode.PercentOutput, driveRight*1);
                break;
        }

        shuffleBoardDisplay(buttonLevel, "Gear Mode");
    }

    public void armMovement() {
        if (shoulderLeft) {
            armMotorA.set(-0.2);
        }
        else if (shoulderRight) {
            armMotorA.set(0.2);
        }
        else{
            armMotorA.set(0);
        }
    }

    public void armWheels() {
        if (buttonX) {
            armMotorB.set(1.0);
        }
        else if (buttonY) {
            armMotorB.set(-1.0);
        }
        else {
            armMotorB.set(0);
        }
    }

    //Turns Motor Power to 0
    public void motorsDisabled() {
        leftMotorFront.setInverted(true);
        leftMotorBack.setInverted(true);
        rightMotorFront.setInverted(false);
        rightMotorBack.setInverted(false);
        
        //Drivetrain Logic
        double forward = right + left;
        double driveLeft = forward - stickInput;
        double driveRight = forward + stickInput;

        leftMotorFront.set(ControlMode.PercentOutput, driveLeft*0);
        leftMotorBack.set(ControlMode.PercentOutput, driveLeft*0);
        rightMotorFront.set(ControlMode.PercentOutput, driveRight*0);
        rightMotorBack.set(ControlMode.PercentOutput, driveRight*0);
    }

    public void motorsForward(float input) {
        leftMotorFront.setInverted(true);
        leftMotorBack.setInverted(true);
        rightMotorFront.setInverted(false);
        rightMotorBack.setInverted(false);

        leftMotorFront.set(ControlMode.PercentOutput, input);
        leftMotorBack.set(ControlMode.PercentOutput, input);
        rightMotorFront.set(ControlMode.PercentOutput, input);
        rightMotorBack.set(ControlMode.PercentOutput, input);
    }

    //Sends Motor Data to Dashboard
    public void motors() {
        double motorPowerLF = leftMotorFront.getBusVoltage();
        double motorPowerLB = leftMotorBack.getBusVoltage();
        double motorPowerRF = rightMotorFront.getBusVoltage();
        double motorPowerRB = rightMotorBack.getBusVoltage();
    
        shuffleBoardDisplay(motorPowerLF, "Front Left Motor");
        shuffleBoardDisplay(motorPowerLB, "Front Right Motor");
        shuffleBoardDisplay(motorPowerRF, "Back Left Motor");
        shuffleBoardDisplay(motorPowerRB, "Back Right Motor");
    }

    public void motorsLeft(float input) {
        leftMotorFront.setInverted(true);
        leftMotorBack.setInverted(true);

        leftMotorFront.set(ControlMode.PercentOutput, input);
        leftMotorBack.set(ControlMode.PercentOutput, input);
    }

    public void motorsRight(float input) {
        rightMotorFront.setInverted(false);
        rightMotorBack.setInverted(false);

        rightMotorFront.set(ControlMode.PercentOutput, input);
        rightMotorBack.set(ControlMode.PercentOutput, input);
    }

    public void acceleration() {
        double xAccel = accelerometer.getX();
        double yAccel = accelerometer.getY();
    
        // Calculates the jerk in the X and Y directions
        // Divides by .02 because default loop timing is 20ms
        double xJerk = (xAccel - prevXAccel)/.02;
        double yJerk = (yAccel - prevYAccel)/.02;
    
        prevXAccel = xAccel;
        prevYAccel = yAccel;
    
        shuffleBoardDisplay(xJerk, "x Jerk");
        shuffleBoardDisplay(yJerk, "y Jerk");
    }


    //Function to Display Data on ShuffleBoard 
    public void shuffleBoardDisplay(double shuffleBoardInput, String inputName) {
        SmartDashboard.putNumber(inputName, shuffleBoardInput);
    }

    public void autonomousInitialization() {
         autoStart = Timer.getFPGATimestamp();
         goForAuto = true;
    }

    //public void autonomousDriveTest() {
    //     double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    //     if(goForAuto) {
    //         if (autoTimeElapsed < 1) {
    //             motorsForward(0.3F);
    //         } else {
    //             motorsDisabled();
    //         }
    //     }
    // }

    //public void autoForward(double input) {
    //     double forwardInSecond = 3;
    //     double onTime = input/forwardInSecond;
    //     double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    //     if(goForAuto) {
    //         if (autoTimeElapsed <= onTime) {
    //             motorsForward(0.2F);
    //         } else {
    //             motorsDisabled();
    //         }
    //     }
    // }

    //public void autoTurn(double input) {
    //     double angleInSecond = 10;
    //     double turnAngle = input/angleInSecond;
    //     double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    //     if (input > 0) {
    //         if (autoTimeElapsed <= turnAngle) {
    //             motorsRight(0.2F);
    //         }
    //     } else if (input < 0) {
    //         if (autoTimeElapsed <= turnAngle) {
    //             motorsLeft(0.2F);
    //         }
    //     }
    // }

    public void autonomousTest() {
    //     autoForward(3);
    //     autoForward(-6);
    // }
        double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
        if(goForAuto){
        //series of timed events making up the flow of auto
            if(autoTimeElapsed < 1){
                //spit out the ball for three seconds
                armMotorA.set(0);
                armMotorB.set(0);
            }
            else if(autoTimeElapsed < 2){
                armMotorA.set(0);
                armMotorB.set(0);
            }else if(autoTimeElapsed < 2.5){
                //stop spitting out the ball and drive backwards *slowly* for three seconds
                armMotorA.set(0);
                armMotorB.set(0);
                leftMotorFront.set(ControlMode.PercentOutput,-0.3);
                leftMotorBack.set(ControlMode.PercentOutput,-0.3);
                rightMotorFront.set(ControlMode.PercentOutput,-0.3);
                rightMotorBack.set(ControlMode.PercentOutput,-0.3);
            }else if(autoTimeElapsed < 3){
                armMotorA.set(0);
                armMotorB.set(0);
                leftMotorFront.set(ControlMode.PercentOutput,0.0);
                leftMotorBack.set(ControlMode.PercentOutput,0.0);
                rightMotorFront.set(ControlMode.PercentOutput,0.0);
                rightMotorBack.set(ControlMode.PercentOutput,0.0);
            }else if(autoTimeElapsed < 5){
                //do nothing for the rest of auto
                armMotorA.set(0);
                armMotorB.set(0);
                leftMotorFront.set(ControlMode.PercentOutput,0.5);
                leftMotorBack.set(ControlMode.PercentOutput,0.5);
                rightMotorFront.set(ControlMode.PercentOutput,0.5);
                rightMotorBack.set(ControlMode.PercentOutput,0.5);
            }else{
                armMotorA.set(0);
                armMotorB.set(0);
                leftMotorFront.set(ControlMode.PercentOutput,0.0);
                leftMotorBack.set(ControlMode.PercentOutput,0.0);
                rightMotorFront.set(ControlMode.PercentOutput,0.0);
                rightMotorBack.set(ControlMode.PercentOutput,0.0);
            }
        }   
    }
}