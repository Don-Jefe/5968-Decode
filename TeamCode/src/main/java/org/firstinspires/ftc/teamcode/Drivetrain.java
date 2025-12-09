package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {

    // Drive motors
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    // Subsystems
    public final DcMotor intake;
    public final DcMotor feeder;
    private final DcMotorEx flywheel;

    // Shooter blocker
    public final DcMotor blocker;

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
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        blocker = hardwareMap.get(DcMotor.class, "blocker");

        // Motor directions
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        // Braking mode for cleaner control
        setBrakeMode(leftFront, leftBack, rightFront, rightBack, intake, feeder, blocker);

        // -------------------------
        // SHOOTER BLOCKER FIX
        // -------------------------
        blocker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Always begin in RUN_TO_POSITION after TeleOp resets it
        blocker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blocker.setTargetPosition(33);
        blocker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blocker.setPower(0.5); // hold the closed position

    }

    // ------------------ DRIVE ------------------

    public void drive(double y, double x, double rx, boolean isFieldCentric) {

        if (isFieldCentric) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Correct field-oriented math
            double rotX = x * Math.cos(heading) + y * Math.sin(heading);
            double rotY = -x * Math.sin(heading) + y * Math.cos(heading);

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

    public void resetIMU() {
        imu.resetYaw();
    }

    public boolean toggleFieldCentric(boolean currentState) {
        return !currentState;
    }

    private void setMotorPowers(double lf, double lb, double rf, double rb) {
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

    // ------------------ SHOOTER ------------------

    public void openShoot() {
        blocker.setTargetPosition(350); // open fully
        blocker.setPower(1);
    }

    public void closeShoot() {
        blocker.setTargetPosition(140);   // closed
        blocker.setPower(1);
    }

    public void stopShoot() {
        blocker.setPower(0);
    }

    // ------------------ SUBSYSTEMS ------------------

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void intakeOut() {
        setIntakePower(-1);
    }

    public void intakeStop() {
        setIntakePower(0);
    }

    public void setFeederPower(double power) {
        feeder.setPower(power);
    }

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    public void setFlywheelRPM(double rpm) {
        // Convert RPM → ticks per second
        double ticksPerSecond = (rpm / 60.0) * 28.0;

        // Ensure motor is in correct mode
        if (flywheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set the velocity (ticks/second)
        flywheel.setVelocity(ticksPerSecond);
    }

}
