package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
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

    // IMU variable
    private final IMU imu;

    /**
     * Constructor to initialize the drivetrain.
     * @param hardwareMap The hardware map from the OpMode.
     */
    public Drivetrain(HardwareMap hardwareMap) {
        // --- INITIALIZATION ---

        // Initialize drive motors from the hardware map
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lr");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rr");

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

        // Define the orientation of the robot's control hub
        // TODO: Adjust these values based on how the control hub is mounted on your robot.
        // For example, if the logo is facing up and the USB ports are facing forward:
        //      RevHubOrientationOnRobot.LogoFacingDirection.UP,
        //      RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
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

    /**
     * Sets the power for each of the four drive motors.
     */
    public void setMotorPowers(double lf, double lb, double rf, double rb) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    /**
     * Resets the IMU's yaw angle to zero. This is useful for recalibrating the "forward" direction
     * for field-centric drive.
     */
    public void resetIMU() {
        imu.resetYaw();
    }
}
