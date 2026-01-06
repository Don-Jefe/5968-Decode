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
            double y = gamepad1.left_stick_y;     // Forward/backward
            double x = -gamepad1.left_stick_x;    // Strafing
            double rx = -gamepad1.right_stick_x;  // Rotation
            drivetrain.drive(y, x, rx, isFieldCentric);

            // Field-centric toggle (press 'A')
            if (gamepad1.a) {
                isFieldCentric = drivetrain.toggleFieldCentric(isFieldCentric);
            }

            // Intake control
            if (gamepad1.left_trigger > 0.5) {
                drivetrain.setIntakePower(0.8);
            } else {
                drivetrain.setIntakePower(0);
            }

            if (gamepad1.leftBumperWasPressed() && drivetrain.getBlockerAngle() == drivetrain.SERVO_TOP_POS)
            {
                drivetrain.setBlockerAngle(drivetrain.SERVO_Bottom_POS);
            } else
            {
                drivetrain.setBlockerAngle(drivetrain.SERVO_TOP_POS);
            }

            if (gamepad1.rightBumperWasPressed())
            {
                drivetrain.setFlywheelRPM(CF.CloseRPM);
            } else
            {
                drivetrain.setFlywheelRPM(0);
            }

            // Telemetry
            telemetry.addData("Driving Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addLine("Press 'A' to toggle mode.");
            telemetry.update();
        }
    }
}