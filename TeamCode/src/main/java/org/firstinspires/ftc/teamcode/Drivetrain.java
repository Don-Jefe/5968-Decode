package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drivetrain class for a mecanum-drive robot.
 * Handles motor control, IMU orientation, and subsystems (intake).
 */
public class Drivetrain {

    // Drive motors
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    // Subsystems
    private final DcMotor intake;

    // Sensors
    private final IMU imu;

    public Drivetrain(HardwareMap hardwareMap) {
        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lr");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rr");

        // Subsystems
        intake = hardwareMap.get(DcMotor.class, "intake");

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        // Motor directions
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake mode
        setBrakeMode(leftFront, leftBack, rightFront, rightBack, intake);
    }

    /**
     * Standard mecanum drive method.
     * @param y Forward/backward input (-1 to 1)
     * @param x Strafe input (-1 to 1)
     * @param rx Rotation input (-1 to 1)
     * @param isFieldCentric Enables field-centric control if true
     * Field Centric does not work and the code is garbage
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
     * Toggles field-centric mode and returns the new state.
     * @param currentState Current field-centric state
     * @return New field-centric state
     */
    public boolean toggleFieldCentric(boolean currentState) {
        return !currentState;  // Simple toggle
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    // Motor control helpers
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

    // Intake control
    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void intakeOut() {
        setIntakePower(-1);
    }

    public void intakeStop() {
        setIntakePower(0);
    }
}