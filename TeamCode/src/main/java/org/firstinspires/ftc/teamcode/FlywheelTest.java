package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Dual Flywheel Velocity Test")
public class FlywheelTest extends OpMode {

    /* ================= MOTORS ================= */

    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;

    /* ================= RPM CONTROL ================= */

    private double targetRPM = 0;
    private double rampedRPM = 0;

    public static double MAX_RPM = 6000;
    public static double RPM_STEP = 100;
    public static double RPM_ACCEL = 8000;   // RPM per second up
    public static double RPM_DECEL = 12000;  // RPM per second down

    private double lastLoopTime;

    /* ================= CONSTANTS ================= */

    // goBILDA 5202/312RPM motor = 28 ticks per rev
    private static final double TICKS_PER_REV = 28.0;

    /* ================= INIT ================= */

    @Override
    public void init() {

        flywheel  = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reverse second motor if wheels face opposite directions
        flywheel2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Dual Flywheel Velocity Test Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        lastLoopTime = getRuntime();
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {

        /* ===== Time Delta ===== */
        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;

        /* ===== RPM Control ===== */

        if (gamepad1.dpad_up) {
            targetRPM += RPM_STEP;
        }

        if (gamepad1.dpad_down) {
            targetRPM -= RPM_STEP;
        }

        if (gamepad1.a) {
            targetRPM = 0;
        }

        targetRPM = clamp(targetRPM, -6000, MAX_RPM);

        /* ===== Ramp Logic ===== */

        if (rampedRPM < targetRPM) {
            rampedRPM += RPM_ACCEL * dt;
            if (rampedRPM > targetRPM) rampedRPM = targetRPM;
        } else {
            rampedRPM -= RPM_DECEL * dt;
            if (rampedRPM < targetRPM) rampedRPM = targetRPM;
        }

        /* ===== Set Velocity (Vibecode Style) ===== */

        double ticksPerSecond = rpmToTicksPerSecond(rampedRPM);

        flywheel.setVelocity(ticksPerSecond);
        flywheel2.setVelocity(ticksPerSecond);

        /* ===== Telemetry ===== */

        telemetry.addLine("===== TARGET =====");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Ramped RPM", rampedRPM);

        telemetry.addLine("===== MOTOR 1 =====");
        telemetry.addData("Velocity (ticks/s)", flywheel.getVelocity());
        telemetry.addData("RPM", getActualRPM(flywheel));

        telemetry.addLine("===== MOTOR 2 =====");
        telemetry.addData("Velocity (ticks/s)", flywheel2.getVelocity());
        telemetry.addData("RPM", getActualRPM(flywheel2));

        telemetry.update();
    }

    /* ================= HELPERS ================= */

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    private double getActualRPM(DcMotorEx motor) {
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}