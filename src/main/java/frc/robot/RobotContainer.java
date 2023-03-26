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
        if (buttonLevel >= 3) {
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
                leftMotorFront.set(ControlMode.PercentOutput, driveLeft*1);
                leftMotorBack.set(ControlMode.PercentOutput, driveLeft*1);
                rightMotorFront.set(ControlMode.PercentOutput, driveRight*1);
                rightMotorBack.set(ControlMode.PercentOutput, driveRight*1);
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
            armMotorB.set(-0.75);
        }

        
        else if (shoulderRight) {
            armMotorB.set(0.8);
        }
        else{
            armMotorB.set(0);
        }
    }

    public void armWheels() {
        if (buttonX) {
            armMotorA.set(1.0);
        }
        else if (buttonY) {
            armMotorA.set(-1.0);
        }
        else {
            armMotorA.set(0);
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

    public void autonomousForward(double time, double strength) {
        double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
        if (goForAuto) {
            if (autoTimeElapsed < time) {
                leftMotorFront.set(ControlMode.PercentOutput,strength);
                leftMotorBack.set(ControlMode.PercentOutput,strength);
                rightMotorFront.set(ControlMode.PercentOutput,strength);
                rightMotorBack.set(ControlMode.PercentOutput,strength);
            } 
            // else {
            //     leftMotorFront.set(ControlMode.PercentOutput,0);
            //     leftMotorBack.set(ControlMode.PercentOutput,0);
            //     rightMotorFront.set(ControlMode.PercentOutput,0);
            //     rightMotorBack.set(ControlMode.PercentOutput,0);
            // }
        }
    }

    public void autonomousTurn(double time, String direction) {
        double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
        if (goForAuto) {
            if (autoTimeElapsed > time && direction == "right") {
                leftMotorFront.set(ControlMode.PercentOutput,0.2);
                leftMotorBack.set(ControlMode.PercentOutput,0.2);
            } else if (autoTimeElapsed > time && direction == "left") {
                rightMotorFront.set(ControlMode.PercentOutput,0.2);
                rightMotorBack.set(ControlMode.PercentOutput,0.2);
            } else {
                leftMotorFront.set(ControlMode.PercentOutput,0);
                leftMotorBack.set(ControlMode.PercentOutput,0);
                rightMotorFront.set(ControlMode.PercentOutput,0);
                rightMotorBack.set(ControlMode.PercentOutput,0);
            }
        }
    }

    public void autonomousCancel() {
        if (goForAuto) {
            leftMotorFront.set(ControlMode.PercentOutput,0);
            leftMotorBack.set(ControlMode.PercentOutput,0);
            rightMotorFront.set(ControlMode.PercentOutput,0);
            rightMotorBack.set(ControlMode.PercentOutput,0);
        }
    }

    public void autonomousTest() {
        if (goForAuto) {
            autonomousForward(1 , 0.2);
            autonomousCancel();
            autonomousForward(2 , -0.2);
            autonomousCancel();
            autonomousTurn(3, "left");
            autonomousCancel();
        }
    }

    public void autoFinal() {
        if (goForAuto) {
            autonomousForward(1, -0.2);
            autonomousCancel();
            autonomousForward(3, 0.2);
            autonomousCancel();
        }
    }

    public void autonomousTestOldWithCharge() {
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
            }else if(autoTimeElapsed < 3){
                //stop spitting out the ball and drive backwards *slowly* for three seconds
                armMotorA.set(0);
                armMotorB.set(0);
                leftMotorFront.set(ControlMode.PercentOutput,-0.3);
                leftMotorBack.set(ControlMode.PercentOutput,-0.3);
                rightMotorFront.set(ControlMode.PercentOutput,-0.3);
                rightMotorBack.set(ControlMode.PercentOutput,-0.3);
            // }else if(autoTimeElapsed < 3){
            //     armMotorA.set(0);
            //     armMotorB.set(0);
            //     leftMotorFront.set(ControlMode.PercentOutput,0.0);
            //     leftMotorBack.set(ControlMode.PercentOutput,0.0);
            //     rightMotorFront.set(ControlMode.PercentOutput,0.0);
            //     rightMotorBack.set(ControlMode.PercentOutput,0.0);
            }else if(autoTimeElapsed < 4.76){
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

    public void autonomousTestOldNoCharge() {
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
            }else if(autoTimeElapsed < 3){
                //stop spitting out the ball and drive backwards *slowly* for three seconds
                armMotorA.set(0);
                armMotorB.set(0);
                leftMotorFront.set(ControlMode.PercentOutput,-0.3);
                leftMotorBack.set(ControlMode.PercentOutput,-0.3);
                rightMotorFront.set(ControlMode.PercentOutput,-0.3);
                rightMotorBack.set(ControlMode.PercentOutput,-0.3);
            // }else if(autoTimeElapsed < 3){
            //     armMotorA.set(0);
            //     armMotorB.set(0);
            //     leftMotorFront.set(ControlMode.PercentOutput,0.0);
            //     leftMotorBack.set(ControlMode.PercentOutput,0.0);
            //     rightMotorFront.set(ControlMode.PercentOutput,0.0);
            //     rightMotorBack.set(ControlMode.PercentOutput,0.0);
            }else if(autoTimeElapsed < 6){
                //do nothing for the rest of auto
                armMotorA.set(0); 
                armMotorB.set(0);
                
                leftMotorFront.set(ControlMode.PercentOutput,0.3);
                leftMotorBack.set(ControlMode.PercentOutput,0.3);
                rightMotorFront.set(ControlMode.PercentOutput,0.3);
                rightMotorBack.set(ControlMode.PercentOutput,0.3);
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