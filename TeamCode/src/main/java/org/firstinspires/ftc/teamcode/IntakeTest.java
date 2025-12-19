package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Main TeleOp class.
 * Controls the robot during driver operation using the Drivetrain subsystem.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Decode - Intake-Test")
public class IntakeTest extends LinearOpMode {

    private DcMotor intake;
    double power = -.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();
        intake = hardwareMap.get(DcMotor.class, "intake");
        waitForStart();
        if (isStopRequested()) return;

        // Main loop
        while (opModeIsActive()) {
           if (gamepad1.triangleWasPressed()) {
               intake.setPower(power);
           } else if(gamepad1.crossWasPressed()) {
               intake.setPower(0);
           } else if (gamepad1.squareWasPressed()) {
               power+= 0.01;
           } else if (gamepad1.circleWasPressed()) {
               power -= 0.01;
           }
           telemetry.addData("intake Power: ", power);
            telemetry.update();
        }
    }
}