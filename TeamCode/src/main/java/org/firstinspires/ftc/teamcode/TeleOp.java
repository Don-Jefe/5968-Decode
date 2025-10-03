package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is the main TeleOp class.
 * It uses a Drivetrain object to handle all movement and hardware interaction,
 * keeping this class focused on driver controls and the OpMode flow.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "(Jeff is awesome and made the best TeleOp) Tele-Op")
public class TeleOp extends LinearOpMode {

    private Drivetrain drivetrain;

    // Variable to track the current driving mode
    private boolean isFieldCentric = true;

    // Variables to manage the toggle button state
    private boolean yButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---
        drivetrain = new Drivetrain(hardwareMap);

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();

        // Wait for the OpMode to be started by the driver
        waitForStart();

        if (isStopRequested()) return;

        // Reset the IMU heading at the start of the match
        drivetrain.resetIMU();

        // --- MAIN LOOP ---
        while (opModeIsActive()) {

            // --- TOGGLE FIELD-CENTRIC MODE ---
            // Use a simple boolean flag to ensure the toggle happens only once per press.
            if (gamepad1.y && !yButtonPressed) {
                isFieldCentric = !isFieldCentric;
                if(isFieldCentric) {
                    // Reset the IMU heading when switching back to field-centric
                    drivetrain.resetIMU();
                }
            }
            yButtonPressed = gamepad1.y;

            // --- GAMEPAD INPUTS ---
            // The Y-axis is inverted on gamepads, so we negate it.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // --- DRIVE THE ROBOT ---
            // Pass the joystick values and the current drive mode to the drivetrain object.
            drivetrain.drive(y, x, rx, isFieldCentric);

            // --- TELEMETRY ---
            telemetry.addData("Driving Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addLine("Press 'Y' to toggle mode.");
            telemetry.update();
        }
    }
}
