package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is the main TeleOp class.
 * It uses a Drivetrain object to handle all movement and hardware interaction,
 * keeping this class focused on driver controls and the OpMode flow.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "(Jeff is dope and made the best TeleOp) Tele-Op")
public class TeleOp extends LinearOpMode {

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // Variable to track the current driving mode
    private boolean isFieldCentric = false;
    private boolean flyWheelOn = false;
    private boolean feederOn = false;
    // Variables to manage the toggle button state
    private boolean yButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---
        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();
        // Wait for the OpMode to be started by the driver
        waitForStart();

        if (isStopRequested()) return;

        // Reset the IMU heading at the start of the match
        drivetrain.resetIMU();

        // --- MAIN LOOP ---
        while (opModeIsActive()) {

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // --- TOGGLE FIELD-CENTRIC MODE ---
            // Use a simple boolean flag to ensure the toggle happens only once per press.

            yButtonPressed = gamepad1.y;

            // --- GAMEPAD INPUTS ---
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            Object rotation = gamepad1.right_stick_x * 1;

            // --- DRIVE THE ROBOT ---
            // Pass the joystick values and the current drive mode to the drivetrain object.
            drivetrain.drive(y, x, rx, isFieldCentric);
            if (gamepad1.a) {
                isFieldCentric = !isFieldCentric;
            }

            // --- TELEMETRY ---
            telemetry.addData("Driving Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addLine("Press 'Y' to toggle mode.");


             if (gamepad1.left_trigger >= 0.5) {
                drivetrain.intakeOut();
            } else {
                drivetrain.intakeStop();
            }
            if (gamepad1.right_trigger >= 0.5) {
                    drivetrain.setFlywheelPower(-0.6);
            } else{
                drivetrain.setFlywheelPower(0);
            }

            if (gamepad1.left_bumper) {
                drivetrain.lowerFeeder.setPower(1);
                drivetrain.upperFeeder.setPower(1);
            } else if(gamepad1.right_bumper) {
                drivetrain.lowerFeeder.setPower(-1);
                drivetrain.upperFeeder.setPower(-1);
            }
            else {
                drivetrain.setFeederPower(0);
            }

            telemetry.update();
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

        }
    }
}
