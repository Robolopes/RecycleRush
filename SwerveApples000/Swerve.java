/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 * Code from Chief Delphi: http://www.chiefdelphi.com/forums/showthread.php?t=117099
 */


import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 * 
 */
public class Drive extends Subsystem {
    //declare the steering pods and shifter valve

    private Pod frontLeft, frontRight, rearLeft, rearRight;
    private DoubleSolenoid shift;

    public Drive() {
        //set up the steering pods with the correct sensors and controllers
        shift = new DoubleSolenoid(RobotMap.Solenoid.DRIVE_SHIFT_HIGH, RobotMap.Solenoid.DRIVE_SHIFT_LOW);
        frontLeft = new Pod(RobotMap.PWM.DRIVE_FRONT_LEFT,
                RobotMap.PWM.DRIVE_FRONT_LEFT_STEERING,
                RobotMap.DIO.DRIVE_FRONT_LEFT_ENC_A,
                RobotMap.DIO.DRIVE_FRONT_LEFT_ENC_B, 1);
        frontRight = new Pod(RobotMap.PWM.DRIVE_FRONT_RIGHT,
                RobotMap.PWM.DRIVE_FRONT_RIGHT_STEERING,
                RobotMap.DIO.DRIVE_FRONT_RIGHT_ENC_A,
                RobotMap.DIO.DRIVE_FRONT_RIGHT_ENC_B, 2);
        rearLeft = new Pod(RobotMap.PWM.DRIVE_REAR_LEFT,
                RobotMap.PWM.DRIVE_REAR_LEFT_STEERING,
                RobotMap.DIO.DRIVE_REAR_LEFT_ENC_A,
                RobotMap.DIO.DRIVE_REAR_LEFT_ENC_B, 3);
        rearRight = new Pod(RobotMap.PWM.DRIVE_REAR_RIGHT,
                RobotMap.PWM.DRIVE_REAR_RIGHT_STEERING,
                RobotMap.DIO.DRIVE_REAR_RIGHT_ENC_A,
                RobotMap.DIO.DRIVE_REAR_RIGHT_ENC_B, 4);
    }

    public void teleopDrive() {
        //get values from joysticks
        double x, y, rotate;
        boolean isLowGear, isHighGear;
        x = CommandBase.oi.stick1.getRawAxis(RobotMap.Control.DRIVE_CONTROLLER_AXIS_1);
        y = -CommandBase.oi.stick1.getRawAxis(RobotMap.Control.DRIVE_CONTROLLER_AXIS_2);
        rotate = CommandBase.oi.stick2.getRawAxis(RobotMap.Control.DRIVE_CONTROLLER_AXIS_1);
        isLowGear = CommandBase.oi.stick1.getRawButton(RobotMap.Control.DRIVE_CONTROLLER_SHIFT_LOW);
        isHighGear = CommandBase.oi.stick1.getRawButton(RobotMap.Control.DRIVE_CONTROLLER_SHIFT_HIGH);
        //calculate angle/speed setpoints using 28 by 38 inch robot 
        double L = 28, W = 38;
        double R = Math.sqrt((L * L) + (W * W));
        double A = x - rotate * (L / R);
        double B = x + rotate * (L / R);
        double C = y - rotate * (W / R);
        double D = y + rotate * (W / R);
        //find wheel speeds
        double frontRightWheelSpeed = Math.sqrt((B * B) + (C * C));
        double frontLeftWheelSpeed = Math.sqrt((B * B) + (D * D));
        double rearLeftWheelSpeed = Math.sqrt((A * A) + (D * D));
        double rearRightWheelSpeed = Math.sqrt((A * A) + (C * C));
        //normalize wheel speeds
        double max = frontRightWheelSpeed;
        if (frontLeftWheelSpeed > max) {
            max = frontLeftWheelSpeed;
        }
        if (rearLeftWheelSpeed > max) {
            max = rearLeftWheelSpeed;
        }
        if (rearRightWheelSpeed > max) {
            max = rearRightWheelSpeed;
        }
        if (max > 1) { 
            // Added if based on comment by Ether
            // Max is more than 1.0, so normalize.
            frontRightWheelSpeed /= max;
            frontLeftWheelSpeed /= max;
            rearLeftWheelSpeed /= max;
            rearRightWheelSpeed /= max;
        }
        //find steering angles
        double frontRightSteeringAngle = MathUtils.atan2(B, C)*180/Math.PI;
        double frontLeftSteeringAngle = MathUtils.atan2(B, D)*180/Math.PI;
        double rearLeftSteeringAngle = MathUtils.atan2(A, D)*180/Math.PI;
        double rearRightSteeringAngle = MathUtils.atan2(A, C)*180/Math.PI;
        //set shifter
        if(isLowGear){
            shift.set(DoubleSolenoid.Value.kForward);
        }
        if(isHighGear){
            shift.set(DoubleSolenoid.Value.kReverse);
        }
        //set pods
        frontRight.setSteeringAngle(frontRightSteeringAngle);
        frontRight.setWheelSpeed(frontRightWheelSpeed);
        frontLeft.setSteeringAngle(frontLeftSteeringAngle);
        frontLeft.setWheelSpeed(frontLeftWheelSpeed);
        rearLeft.setSteeringAngle(rearLeftSteeringAngle);
        rearLeft.setWheelSpeed(rearLeftWheelSpeed);
        rearRight.setSteeringAngle(rearRightSteeringAngle);
        rearRight.setWheelSpeed(rearRightWheelSpeed);

    }

    private class Pod implements PIDOutput, PIDSource {

        private Encoder steeringEnc;
        private SpeedController drive;
        private SpeedController steer;
        private PIDController pid;

        public Pod(int drivePWM, int steeringPWM, int steeringEncA,
                int steeringEncB, int podNumber) {
            steeringEnc = new Encoder(steeringEncA, steeringEncB);
            steeringEnc.setDistancePerPulse(RobotMap.Constants.STEERING_ENC_REVOLUTIONS_PER_PULSE);
            drive = new Victor(drivePWM);
            steer = new Victor(steeringPWM);
            pid = new PIDController(RobotMap.Constants.STEERING_PID_P,
                    RobotMap.Constants.STEERING_PID_I,
                    RobotMap.Constants.STEERING_PID_D, this, this);
            SmartDashboard.putData("Steering Pod " + podNumber, pid);
            pid.setInputRange(-180, 180);
            pid.setContinuous(true);
            pid.enable();

        }

        public void pidWrite(double output) {
            steer.set(output);
        }

        public double pidGet() {
            return steeringEnc.getDistance();
        }

        public void setSteeringAngle(double angle) {
            pid.setSetpoint(angle);
        }

        public void setWheelSpeed(double speed) {
            drive.set(speed);
        }
    }

    public void initDefaultCommand() {
    }
}