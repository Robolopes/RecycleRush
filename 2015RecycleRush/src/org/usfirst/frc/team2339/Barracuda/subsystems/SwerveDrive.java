package org.usfirst.frc.team2339.Barracuda.subsystems;

/*
 * Add a swerve mode to RobotDrive
 * Code from Chief Delphi: http://www.chiefdelphi.com/forums/showthread.php?t=117099
 */


import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.RectangularCoordinates;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.RobotMotion;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.VelocityPolar;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * 
 */
public class SwerveDrive extends Subsystem {

	/*
	 * Wheel are numbered in counter-clockwise order when viewed from top of robot.
	 * For a typical four wheel configuration the front right wheel is Number 0.
	 * This follows the scheme in Ether's derivation of swerve inverse kinematics.
	 * @See http://www.chiefdelphi.com/media/papers/2426
	 * 
	 * Note: Ether starts numbering at 1, but Java indices start at zero. 
	 *       Thus the wheel indices are one less than Ether's numbers.
	 * 
	 */
    
    protected final SwerveWheelDrive wheels[];
    protected RectangularCoordinates pivot = new RectangularCoordinates(0, 0);

	// Distance of wheel farthest from pivot
    protected double maxWheelRadius = 1;
    
    public SwerveDrive(SwerveWheelDrive wheels[]) {
    	this.wheels = new SwerveWheelDrive[wheels.length];
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		this.wheels[iiWheel] = wheels[iiWheel];
    	}
    	setPivot(new RectangularCoordinates(0, 0));
    }
    
    public double getMaxWheelRadius() {
		return maxWheelRadius;
	}

	public void setMaxWheelRadius() {
		maxWheelRadius = 0;
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		double radius = wheels[iiWheel].getWheelPosition().subtract(pivot).magnitude();
    		if (radius > maxWheelRadius) {
    			maxWheelRadius = radius;
    		}
    	}
	}

    public RectangularCoordinates getPivot() {
		return pivot;
	}

	public void setPivot(RectangularCoordinates pivot) {
		this.pivot = pivot;
		setMaxWheelRadius();
	}
	
	public void resetSteering() {
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		wheels[iiWheel].resetSteering();
    	}
	}
	
	public void enableSteering(boolean enable) {
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		wheels[iiWheel].enableSteering(enable);
    	}
	}
	
    /**
     * Drive in swerve mode with a given wheel speeds and directions.
     * Driving parameters are assumed to be relative to the current robot angle.
     * @param rawVelocities desired speed and direction vectors for each wheel.
     */
    public void swerveDriveRobot(
    		VelocityPolar rawVelocities[]) {
    	
    	double speedMax = 0;
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		if (Math.abs(rawVelocities[iiWheel].speed) > speedMax) {
    			speedMax = rawVelocities[iiWheel].speed;
    		}
    	}
    	
    	if (speedMax > 1.0) {
	    	// Normalize speeds to less than |1.0|
	    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
	    		rawVelocities[iiWheel].speed /= speedMax;
	    	}
    	}
    	
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		wheels[iiWheel].setWheelSanely(rawVelocities[iiWheel]);
    	}
    }

    /**
     * Drive in swerve mode with a given speed and direction.
     * Driving parameters are assumed to be relative to the current robot angle.
     * @param robotVelocity desired speed and direction vector.
     */
    public void swerveDriveRobot(
    		VelocityPolar robotVelocity) {
    	
    	VelocityPolar rawVelocities[] = new VelocityPolar[wheels.length];
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		rawVelocities[iiWheel] = robotVelocity;
    	}
    	
    	swerveDriveRobot(rawVelocities);
    }

    /**
     * Drive in swerve mode with a given speed and rotation.
     * Driving parameters are assumed to be relative to the current robot angle.
     * @param robotMotion desired motion of robot express by strafe, frontBack, and rotation around a pivot point.
     * @param pivot Position of pivot. 
     */
    public void swerveDriveRobot(
    		RobotMotion robotMotion, 
    		RectangularCoordinates pivot) {
    	
    	setPivot(pivot);
    	
    	VelocityPolar rawVelocities[] = new VelocityPolar[wheels.length];
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		rawVelocities[iiWheel] = wheels[iiWheel].calculateWheelVelocity(getPivot(), maxWheelRadius, robotMotion);
    	}
    	
    	swerveDriveRobot(rawVelocities);
    }

    /**
     * Drive in swerve mode with a given speed and rotation.
     * Driving parameters are assumed to be absolute based on a fixed angle, e.g. the field.
     * @param robotMotion desired motion of robot express by strafe, frontBack, and rotation around a pivot point.
     * @param robotAngle Angle (in degrees) of robot relative to fixed angle. Zero degrees means front of robot points in desired direction. 
     *                   Positive is clockwise, negative counter-clockwise. This is probably taken from the gyro.
     * @param pivot Position of pivot. 
     */
    public void swerveDriveAbsolute(
    		RobotMotion robotMotion, 
    		double robotAngle,
    		RectangularCoordinates pivot) {
    	double robotAngleRad = Math.toRadians(robotAngle);
    	RobotMotion relativeMotion = new RobotMotion(
    			robotMotion.strafe * Math.cos(robotAngleRad) - robotMotion.frontBack * Math.sin(robotAngleRad),
    			robotMotion.strafe * Math.sin(robotAngleRad) + robotMotion.frontBack * Math.cos(robotAngleRad), 
    			robotMotion.rotate);
    	this.swerveDriveRobot(relativeMotion, pivot);
    }
    
	/**
	 * Set speed and angle values when joystick in dead band
	 */
	public void setDeadBandValues() {
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
        	wheels[iiWheel].setWheel(new VelocityPolar(0, 0));
    	}
	}
	
    /**
     * Control robot using joystick inputs
     */
    public void swerveDriveTeleop() {
    	
    	RobotMotion robotMotion = new RobotMotion(
    			SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_SIDEWAYS), 
    			-SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK), 
    			SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE));
        
    	if (Math.abs(robotMotion.strafe) > SwerveMap.Control.DRIVE_STICK_DEAD_BAND || 
    			Math.abs(robotMotion.frontBack) > SwerveMap.Control.DRIVE_STICK_DEAD_BAND || 
    			Math.abs(robotMotion.rotate) > SwerveMap.Control.DRIVE_STICK_DEAD_BAND) {
    		robotMotion.rotate *= .5;

    		RectangularCoordinates pivot = new RectangularCoordinates(0, 0);
            if (SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_BUTTON_ROTATE_AROUND_CONTAINER)) {
            	pivot.x = 0.0;
            	pivot.y = SwerveMap.Constants.CONTAINER_CENTER_DISTANCE_FORWARD + 0.5 * SwerveMap.Constants.WHEEL_BASE_LENGTH;
            }
            if (SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_BUTTON_SPEED_SHIFT)) {
        		robotMotion.rotate *= .5;
        		robotMotion.strafe *= .5;
        		robotMotion.frontBack *= .5;
        	}
            
            double robotAngle = 0.0;
            if (SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_BUTTON_ABSOLUTE_GYRO_MODE)) {
                robotAngle = SwerveMap.Control.GYRO.getAngle();
            }
            
            swerveDriveAbsolute(robotMotion, robotAngle, pivot);
    	} else {
    		// Joystick in dead band, set neutral values
    		setDeadBandValues();
    	}
    }
    
    /**
     * Class to store angle and flip together
     * @author emiller
     *
     */
    public class AngleFlip {
    	private double angle;
    	private boolean flip;
    	
    	public AngleFlip() {
    		setAngle(0);
    		setFlip(false);
    	}
    	public AngleFlip(double angle) {
    		this.setAngle(angle);
    		setFlip(false);
    	}
    	public AngleFlip(double angle, boolean flip) {
    		this.setAngle(angle);
    		flip = false;
    	}
		/**
		 * @return the angle
		 */
		public double getAngle() {
			return angle;
		}
		/**
		 * @param angle the angle to set
		 */
		public void setAngle(double angle) {
			this.angle = angle;
		}
		/**
		 * @return the flip
		 */
		public boolean isFlip() {
			return flip;
		}
		/**
		 * @param flip the flip to set
		 */
		public void setFlip(boolean flip) {
			this.flip = flip;
		}
    };
    
    /** 
     * Normalizes an angle in degrees to (-180, 180].
     * @param theta Angle to normalize
     * @return Normalized angle
     */
    public double normalizeAngle(double theta) {
    	while (theta > 180) {
    		theta -= 360;
    	}
    	while (theta < -180) {
    		theta += 360;
    	}
    	return theta;
    }

    /**
     * Compute angle needed to turn and whether or not flip is needed
     * @param currentAngle
     * @param targetAngle
     * @return new angle with flip
     */
    public AngleFlip computeTurnAngle(double currentAngle, double targetAngle) {
    	AngleFlip turnAngle = new AngleFlip(targetAngle - currentAngle, false);
    	if (Math.abs(turnAngle.getAngle()) > 90) {
    		turnAngle.setAngle(normalizeAngle(turnAngle.getAngle() + 180));
    		turnAngle.setFlip(true);
    	}
    	return turnAngle;
    }
    
    /**
     * Compute change angle to get from current to target angle.
     * @param currentAngle Current angle
     * @param targetAngle New angle to change to
     * @return change angle
     */
    public double computeChangeAngle(double currentAngle, double targetAngle) {
    	return computeTurnAngle(currentAngle, targetAngle).getAngle();
    }
    
    /**
     * Scale drive speed based on how far wheel needs to turn
     * @param turnAngle Angle wheel needs to turn (with flip value)
     * @return speed scale factor in range [0, 1]
     */
    public double driveScale(AngleFlip turnAngle) {
    	double scale = 0;
    	if (Math.abs(turnAngle.getAngle()) < 45) {
    		/*
    		 * Eric comment: I don't like the discontinuous nature of this scaling.
    		 * Possible improvements:
    		 *   1) Use cosine(2 * turnAngle)
    		 *   2) Scale any angle < 90.
    		 */
    		scale = Math.cos(Math.toRadians(turnAngle.getAngle()));
    	} else {
    		scale = 0;
    	}
    	if (turnAngle.isFlip()) {
    		scale = -scale;
    	}
    	return scale;
    }
    
    public class Pod implements PIDOutput, PIDSource {

        private Encoder steeringEnc;
        private SpeedController drive;
        private SpeedController steer;
        private PIDController pid;
        private int podNumber;

        public Pod(SpeedController driveController, SpeedController steeringController, int steeringEncA,
                int steeringEncB, int podNumber) {
            steeringEnc = new Encoder(steeringEncA, steeringEncB);
            steeringEnc.setDistancePerPulse(SwerveMap.Constants.STEERING_ENC_DEGREES_PER_PULSE);
            drive = driveController;
            steer = steeringController;
            this.podNumber = podNumber;
            pid = new PIDController(SwerveMap.Constants.STEERING_PID_P,
                    SwerveMap.Constants.STEERING_PID_I,
                    SwerveMap.Constants.STEERING_PID_D, this, this);
            SmartDashboard.putData("Steering Pod " + podNumber, pid);
            pid.setInputRange(-180, 180);
            pid.setContinuous(true);
        }

        public void pidWrite(double output) {
            steer.set(output);
        }

        public double pidGet() {
        	SmartDashboard.putData("Enc " + this.podNumber + " ", steeringEnc);
            return steeringEnc.getDistance();
        }

        public void setSteeringAngle(double angle) {
            pid.setSetpoint(angle);
        }
        public void setWheelSpeed(double speed) {
            drive.set(speed);
       // public void setWheelSpeed(double speed) {
         //   drive.set(.5 * speed);
        }
        
        public void resetAngle() {
        	steeringEnc.reset();
        }
        
        public void pidEnable(boolean enable) {
        	if (enable) {
        		pid.enable();
        	} else {
        		pid.disable();
        	}
        }
    }

    public void initDefaultCommand() {
    }

}