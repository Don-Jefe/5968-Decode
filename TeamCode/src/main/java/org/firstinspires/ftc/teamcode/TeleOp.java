package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Main TeleOp class.
 * Controls the robot during driver operation using the Drivetrain subsystem.
 * Clean structure for easy reading and efficient control.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "(Decode - Tele-Op")
public class TeleOp extends LinearOpMode {

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    private boolean isFieldCentric = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---
        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Reset IMU at the start of the match
        drivetrain.resetIMU();

        // --- MAIN LOOP ---
        while (opModeIsActive()) {

            // Update gamepad states
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // --- DRIVE CONTROLS ---
            double y = gamepad1.left_stick_y;     // Forward/backward
            double x = -gamepad1.left_stick_x;    // Strafing
            double rx = -gamepad1.right_stick_x;  // Rotation

            drivetrain.drive(y, x, rx, isFieldCentric);

            // --- FIELD-CENTRIC TOGGLE (press 'A') ---
            if (currentGamepad1.a && !previousGamepad1.a) {
                isFieldCentric = !isFieldCentric;
            }

            // --- INTAKE CONTROL ---
            if (gamepad1.left_trigger > 0.5) {
                drivetrain.intakeOut();
            } else {
                drivetrain.intakeStop();
            }

            // --- FLYWHEEL CONTROL ---
            if (gamepad1.right_trigger > 0.5) {
                drivetrain.setFlywheelPower(-0.6);
            } else {
                drivetrain.setFlywheelPower(0);
            }

            // --- FEEDER CONTROL ---
            if (gamepad1.left_bumper) {
                drivetrain.lowerFeeder.setPower(1);
                drivetrain.upperFeeder.setPower(1);
            } else if (gamepad1.right_bumper) {
                drivetrain.lowerFeeder.setPower(-1);
                drivetrain.upperFeeder.setPower(-1);
            } else {
                drivetrain.setFeederPower(0);
            }

            // --- TELEMETRY ---
            telemetry.addData("Driving Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addLine("Press 'A' to toggle mode.");
            telemetry.update();

            // Save previous gamepad states
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
        }
    }
}
