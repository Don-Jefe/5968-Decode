package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drivetrain class for a mecanum-drive robot.
 * Handles motor control, IMU orientation, and subsystem motors (intake, feeder, flywheels).
 */
public class Drivetrain {

    // --- Drive Motors ---
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    // --- Subsystems ---
    private final DcMotor intake;
    public final CRServo lowerFeeder;
    public final CRServo upperFeeder;
    private final DcMotor leftFlywheel;
    private final DcMotor rightFlywheel;

    // --- Sensors ---
    private final IMU imu;

    // --- State Variables ---
    private double desiredHeadingDegrees = 0;
    private double currentRx = 0;
    private double currentRotation = 0;

    // --- Constants ---
    private static final int EVENTS_PER_MOTOR_ROTATION = 28;
    private static final double WHEEL_MOTOR_RATIO = 19.2;
    private static final double EVENTS_PER_WHEEL_ROTATION = EVENTS_PER_MOTOR_ROTATION * WHEEL_MOTOR_RATIO;
    private static final double INCHES_PER_WHEEL_ROTATION = 12.3; // adjust as needed

    public Drivetrain(HardwareMap hardwareMap) {
        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lr");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rr");

        // Subsystems
        intake = hardwareMap.get(DcMotor.class, "intake");
        lowerFeeder = hardwareMap.get(CRServo.class, "lowerFeeder");
        upperFeeder = hardwareMap.get(CRServo.class, "upperFeeder");
        leftFlywheel = hardwareMap.get(DcMotor.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotor.class, "rightFlywheel");

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        // Motor direction
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake mode for precision control
        setBrakeMode(leftFront, leftBack, rightFront, rightBack, intake);
    }

    // --- DRIVING LOGIC ---

    /**
     * Standard mecanum drive method.
     * @param y Forward/backward input (-1 to 1)
     * @param x Strafe input (-1 to 1)
     * @param rx Rotation input (-1 to 1)
     * @param isFieldCentric Enables field-centric control if true
     */
    public void drive(double y, double x, double rx, boolean isFieldCentric) {
        if (isFieldCentric) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            x = rotX;
            y = rotY;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lf = (y + x + rx) / denominator;
        double lb = (y - x + rx) / denominator;
        double rf = (y - x - rx) / denominator;
        double rb = (y + x - rx) / denominator;

        setMotorPowers(lf, lb, rf, rb);
    }

    /**
     * Field-centric helper drive (legacy style)
     */
    public void Drive(double sideways, double forward, double rotation, boolean fieldCentric) {
        double rx = rotation;
        if (rotation != 0) {
            currentRotation = Math.abs(rotation);
        } else {
            double rotationNeeded = rotationNeededDegrees();
            if (rotationNeeded != 0 && imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES) <= 5) {
                rx = Math.min(1, Math.abs(rotationNeeded) / 90);
                rx = Math.min(rx, Math.abs(currentRx) + 0.1);
                if (Math.abs(rotationNeeded) >= 1) rx = Math.max(rx, 0.15);
                rx = Math.signum(rotationNeeded) * -rx;
            }
        }

        currentRx = rx;

        double lf, lb, rf, rb;
        if (fieldCentric) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = sideways * Math.cos(-heading) - forward * Math.sin(-heading);
            double rotY = sideways * Math.sin(-heading) + forward * Math.cos(-heading);
            rotX *= 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            lf = (rotY + rotX + rx) / denominator;
            lb = (rotY - rotX + rx) / denominator;
            rf = (rotY - rotX - rx) / denominator;
            rb = (rotY + rotX - rx) / denominator;
        } else {
            double denominator = Math.max(Math.abs(forward) + Math.abs(sideways) + Math.abs(rx), 1);
            lf = (forward + sideways + rx) / denominator;
            lb = (forward - sideways + rx) / denominator;
            rf = (forward - sideways - rx) / denominator;
            rb = (forward + sideways - rx) / denominator;
        }

        setMotorPowers(lf, lb, rf, rb);

        if (currentRotation > 0) {
            currentRotation -= 0.1;
            setHeading(currentHeading());
        }
    }

    // --- HEADING + IMU ---

    public double rotationNeededDegrees() {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double rotationNeeded = desiredHeadingDegrees - botHeading;
        if (rotationNeeded > 180) rotationNeeded -= 360;
        if (rotationNeeded < -180) rotationNeeded += 360;
        return rotationNeeded;
    }

    public void setHeading(double desiredHeadingDegrees) {
        this.desiredHeadingDegrees = desiredHeadingDegrees;
    }

    public double currentHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    // --- MOTOR CONTROL HELPERS ---

    public void setMotorPowers(double lf, double lb, double rf, double rb) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    private void setBrakeMode(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // --- SUBSYSTEM CONTROL ---

    // Flywheel
    public void setFlywheelPower(double power) {
        leftFlywheel.setPower(-power);
        rightFlywheel.setPower(power);
    }

    // Feeders
    public void setFeederPower(double power) {
        lowerFeeder.setPower(power);
        upperFeeder.setPower(power);
    }

    // Intake
    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void intakeIn() {
        setIntakePower(1);
    }

    public void intakeOut() {
        setIntakePower(-1);
    }

    public void intakeStop() {
        setIntakePower(0);
    }

    // Shooting helper
    public void shootBall() {
        setFlywheelPower(1.0);
        setFeederPower(1.0);
    }
}
