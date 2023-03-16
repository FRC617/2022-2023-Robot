package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class RobotContainer {
    VictorSPX leftMotorFront = new VictorSPX(Constants.LEFT_MOTOR_A);
    VictorSPX leftMotorBack = new VictorSPX(Constants.LEFT_MOTOR_B);
    VictorSPX rightMotorFront = new VictorSPX(Constants.RIGHT_MOTOR_A);
    VictorSPX rightMotorBack = new VictorSPX(Constants.RIGHT_MOTOR_B);

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

        buttonA = driverController.getRawButton(Constants.CONTROLLER_BUTTON_A);
        buttonB = driverController.getBButton();
        buttonX = driverController.getXButton();
        buttonY = driverController.getYButton();
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
        if (buttonLevel >= 4) {
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
                leftMotorFront.set(ControlMode.PercentOutput, driveLeft*0);
                leftMotorBack.set(ControlMode.PercentOutput, driveLeft*0);
                rightMotorFront.set(ControlMode.PercentOutput, driveRight*0);
                rightMotorBack.set(ControlMode.PercentOutput, driveRight*0);
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

    public void autonomousDriveTest() {
        double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
        if(goForAuto) {
            if (autoTimeElapsed < 1) {
                motorsForward(0.3F);
            } else {
                motorsDisabled();
            }
        }
    }

    public void autoForward(double input) {
        double forwardInSecond = 3;
        double onTime = input/forwardInSecond;
        double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
        if(goForAuto) {
            if (autoTimeElapsed <= onTime) {
                motorsForward(0.3F);
            } else {
                motorsDisabled();
            }
        }
    }

    public void autoTurn(double input) {
        double angleInSecond = 10;
        double turnAngle = input/angleInSecond;
        double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
        if (input > 0) {
            if (autoTimeElapsed <= turnAngle) {
                motorsRight(0.3F);
            }
        } else if (input < 0) {
            if (autoTimeElapsed <= turnAngle) {
                motorsLeft(0.3F);
            }
        }
    }

    public void autonomousTest() {
        autoTurn(90);
        autoForward(1);
    }
}
