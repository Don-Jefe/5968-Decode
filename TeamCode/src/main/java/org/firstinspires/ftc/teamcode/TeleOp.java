package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Main TeleOp class.
 * Controls the robot during driver operation using the Drivetrain subsystem.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Decode - Tele-Op")
public class TeleOp extends LinearOpMode {

    private Drivetrain drivetrain;
    private boolean isFieldCentric = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        drivetrain = new Drivetrain(hardwareMap);
        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Main loop
        while (opModeIsActive()) {
            // Drive controls
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_y, -gamepad1.right_stick_x, isFieldCentric);

           // mechanism
            drivetrain.updateIntake(gamepad1.left_trigger);
            drivetrain.updateBlocker(gamepad1);
            drivetrain.updateFlywheel(gamepad1);
            drivetrain.updateFeeder(gamepad1);

            // Telemetry
            telemetry.addData("Driving Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addLine("Press 'A' to toggle mode.");
            telemetry.update();
        }
    }
}