package org.usfirst.frc.team2339.Barracuda;

/*
 * Add a swerve mode to RobotDrive
 * Code from Chief Delphi: http://www.chiefdelphi.com/forums/showthread.php?t=117099
 */


import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap;
//import com.sun.squawk.util.MathUtils;
import java.lang.Math;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 * 
 */
public class SwerveDrive extends RobotDrive {
    //declare the steering pods and shifter valve

    protected SpeedController m_frontLeftSteeringMotor;
    protected SpeedController m_frontRightSteeringMotor;
    protected SpeedController m_rearLeftSteeringMotor;
    protected SpeedController m_rearRightSteeringMotor;
    private Pod frontLeft, frontRight, rearLeft, rearRight;
    private DoubleSolenoid shift;

    public SwerveDrive() {
    	super(SwerveMap.PWM.DRIVE_FRONT_LEFT, 
    		  SwerveMap.PWM.DRIVE_REAR_LEFT, 
    		  SwerveMap.PWM.DRIVE_FRONT_RIGHT, 
    		  SwerveMap.PWM.DRIVE_REAR_RIGHT);
        m_frontLeftSteeringMotor = new Talon(SwerveMap.PWM.DRIVE_FRONT_LEFT_STEERING);
        m_rearLeftSteeringMotor = new Talon(SwerveMap.PWM.DRIVE_REAR_LEFT_STEERING);
        m_frontRightSteeringMotor = new Talon(SwerveMap.PWM.DRIVE_FRONT_RIGHT_STEERING);
        m_rearRightSteeringMotor = new Talon(SwerveMap.PWM.DRIVE_REAR_RIGHT_STEERING);
        //set up the steering pods with the correct sensors and controllers
        shift = new DoubleSolenoid(SwerveMap.Solenoid.DRIVE_SHIFT_HIGH, SwerveMap.Solenoid.DRIVE_SHIFT_LOW);
        frontLeft = new Pod(m_frontLeftMotor,
        		m_frontLeftSteeringMotor,
                SwerveMap.DIO.DRIVE_FRONT_LEFT_ENC_A,
                SwerveMap.DIO.DRIVE_FRONT_LEFT_ENC_B, 1);
        frontRight = new Pod(m_frontRightMotor,
        		m_frontRightSteeringMotor,
                SwerveMap.DIO.DRIVE_FRONT_RIGHT_ENC_A,
                SwerveMap.DIO.DRIVE_FRONT_RIGHT_ENC_B, 2);
        rearLeft = new Pod(m_rearLeftMotor,
        		m_rearLeftSteeringMotor,
                SwerveMap.DIO.DRIVE_REAR_LEFT_ENC_A,
                SwerveMap.DIO.DRIVE_REAR_LEFT_ENC_B, 3);
        rearRight = new Pod(m_rearRightMotor,
        		m_rearRightSteeringMotor,
                SwerveMap.DIO.DRIVE_REAR_RIGHT_ENC_A,
                SwerveMap.DIO.DRIVE_REAR_RIGHT_ENC_B, 4);
    }

    public void teleopDrive() {
        //get values from joysticks
        double x, y, rotate;
        boolean isLowGear, isHighGear;
        x = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK);
        y = -SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_SIDEWAYS);
        rotate = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE);
        isLowGear = SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_CONTROLLER_SHIFT_LOW);
        isHighGear = SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_CONTROLLER_SHIFT_HIGH);
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
        double frontRightSteeringAngle = Math.atan2(B, C)*180/Math.PI;
        double frontLeftSteeringAngle = Math.atan2(B, D)*180/Math.PI;
        double rearLeftSteeringAngle = Math.atan2(A, D)*180/Math.PI;
        double rearRightSteeringAngle = Math.atan2(A, C)*180/Math.PI;
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

        public Pod(SpeedController driveController, SpeedController steeringController, int steeringEncA,
                int steeringEncB, int podNumber) {
            steeringEnc = new Encoder(steeringEncA, steeringEncB);
            steeringEnc.setDistancePerPulse(SwerveMap.Constants.STEERING_ENC_REVOLUTIONS_PER_PULSE);
            drive = driveController;
            steer = steeringController;
            pid = new PIDController(SwerveMap.Constants.STEERING_PID_P,
                    SwerveMap.Constants.STEERING_PID_I,
                    SwerveMap.Constants.STEERING_PID_D, this, this);
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