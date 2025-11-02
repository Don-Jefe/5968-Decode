package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drivetrain class to manage the mecanum drive hardware and logic.
 * This includes motor and IMU initialization, and drive calculations.
 */
public class Drivetrain {

    // Drive motor variables
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    // Intake variables
    private final DcMotor intake;
    public final CRServo lowerFeeder;
    public final CRServo upperFeeder;

    Gamepad currentGamepad1 = new Gamepad();
    // Out-take Variables
    private final DcMotor rightFlywheel;
    private final DcMotor leftFlywheel;

    // IMU variable
    private final IMU imu;


    private int eventsPerMotorRotation = 28;
    private double wheelMotorRatio = 19.2;
    private double eventsPerWheelRotation = eventsPerMotorRotation * wheelMotorRatio;
    private double inchesPerWheelRotation = 12.3;
    private double desiredHeadingDegrees = 0;
    private double currentrx = 0;
    private double currentRotation = 0;


    /**
     * Constructor to initialize the drivetrain.
     * @param hardwareMap The hardware map from the OpMode.
     */



    public Drivetrain(HardwareMap hardwareMap) {
        // --- INITIALIZATION ---

        // Initialize drive motors from the hardware map
        leftFront = hardwareMap.get(DcMotor.class, "lf"); // left front motor
        leftBack = hardwareMap.get(DcMotor.class, "lr"); // left rear motor
        rightFront = hardwareMap.get(DcMotor.class, "rf"); // right front motor
        rightBack = hardwareMap.get(DcMotor.class, "rr"); // right rear motor
        intake = hardwareMap.get(DcMotor.class, "intake"); // intake motor
        // Configure TS bellow on the bot
        lowerFeeder = hardwareMap.get(CRServo.class, "lowerFeeder"); //  control port: 0
        upperFeeder = hardwareMap.get(CRServo.class, "upperFeeder"); // control port: 1
        leftFlywheel = hardwareMap.get(DcMotor.class, "leftFlywheel"); // left Flywheel motor extension 3 or 4
        rightFlywheel = hardwareMap.get(DcMotor.class, "rightFlywheel"); // right Flywheel motor


        // Initialize the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        // Reverse the right side motors
        // On many robots, the motors on one side need to be reversed for intuitive control
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor behavior when power is zero (BRAKE holds the position)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Define the orientation of the robot's control hub
        // TODO: Adjust these values based on how the control hub is mounted on your robot.
        // For example, if the logo is facing up and the USB ports are facing forward:
        //      RevHubOrientationOnRobot.LogoFacingDirection.UP,
        //      RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        // Initialize the IMU with the specified parameters
        imu.initialize(parameters);
    }

    /**
     * Drives the robot using mecanum wheels with an option for field-centric control.
     * @param y The forward/backward input from the joystick (-1 to 1).
     * @param x The strafing input from the joystick (-1 to 1).
     * @param rx The rotational input from the joystick (-1 to 1).
     * @param isFieldCentric True for field-centric, false for robot-centric.
     */
    public void drive(double y, double x, double rx, boolean isFieldCentric) {
        if (isFieldCentric) {
            // Read the robot's heading from the IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the joystick inputs by the negative of the robot's heading
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Re-assign the rotated values to be used in the mecanum calculation
            x = rotX;
            y = rotY;
        }

        // --- MECANUM DRIVE CALCULATION ---

        // The denominator ensures that no motor power exceeds 1.0, preserving the mix of inputs
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;

        // --- SET MOTOR POWERS ---
        setMotorPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

//    public void Drive(double sideways, double forward) {
//        Drive(sideways, forward, 0, false);
//    }

    public double RotationNeededDegrees(){
        double botHeadingDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double botRotationNeeded = desiredHeadingDegrees - botHeadingDegrees;
        if (botRotationNeeded > 180) {botRotationNeeded = botRotationNeeded - 360;}
        if (botRotationNeeded < -180) {botRotationNeeded = botRotationNeeded + 360;}
        return botRotationNeeded;
    }

    public void Drive(double sideways, double forward, double rotation, boolean fieldCentric) {
        double rx = rotation;
        if (rotation != (double)0) {
            currentRotation = Math.abs(rotation);
        } else {
            double botRotationNeeded = RotationNeededDegrees();
            if (botRotationNeeded != (double)0) {
                double botPitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
                if (botPitch <= 5) {
                    rx = Math.min(1, (double)(Math.abs(botRotationNeeded)) / 90);
                    rx = Math.min(rx, Math.abs(currentrx) + 0.1);
                    if ((double)(Math.abs(botRotationNeeded)) >= 1) {rx = Math.max(rx, 0.15);}
                    rx = Math.signum(botRotationNeeded) * -rx;
                }
            }
        }
        currentrx = rx;
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        if (fieldCentric) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = sideways * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = sideways * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;
        } else {
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(forward) + Math.abs(sideways) + Math.abs(rx), 1);
            frontLeftPower = (forward + sideways + rx) / denominator;
            backLeftPower = (forward - sideways + rx) / denominator;
            frontRightPower = (forward - sideways - rx) / denominator;
            backRightPower = (forward + sideways - rx) / denominator;
        }
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
        leftBack.setPower(backLeftPower);
        if (currentRotation > 0) {
            currentRotation -= 0.1;
            SetHeading(CurrentHeading());
        }
    }
    public void SetHeading(double desiredHeadingDegrees){
        this.desiredHeadingDegrees = desiredHeadingDegrees;
    }
    public double CurrentHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    /**
     * Sets the power for each of the four drive motors.
     */
    public void setMotorPowers(double lf, double lb, double rf, double rb) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }


    public void setFlywheelPower(double dih) {
        leftFlywheel.setPower(-dih);
        rightFlywheel.setPower(dih);
    }
    public void setFeederPower(double stayTunerBurger) {
        lowerFeeder.setPower(stayTunerBurger);
        upperFeeder.setPower(stayTunerBurger);
    }

    public void shootBall() {
        setFlywheelPower(1.0);
        lowerFeeder.setPower(1.0);
        upperFeeder.setPower(1.0);
    }

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


    /**
     * Resets the IMU's yaw angle to zero. This is useful for recalibrating the "forward" direction
     * for field-centric drive.
     */
    public void resetIMU() {
        imu.resetYaw();
    }
}
