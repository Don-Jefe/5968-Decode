package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Decode - Git Tele-Op")
public class GitTele extends LinearOpMode {

    private Drivetrain drivetrain;
    private double shootingPower = -3340; // -.67
    // ideal close is -3340

    private boolean isFieldCentric = false;
    private boolean controllerIsRed;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------------
        // Initialize drivetrain
        // -------------------------------
        drivetrain = new Drivetrain(hardwareMap);
        telemetry.addLine("Init Complete");
        telemetry.update();

        // -------------------------------
        // BLOCKER CALIBRATION
        // -------------------------------
        // Push blocker "closed" gently



        telemetry.addLine("Blocker calibrated.");
        telemetry.update();


        // Wait for start
        waitForStart();
        if (isStopRequested()) return;
        drivetrain.blocker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.blocker.setPower(-1);
        sleep(500);
        drivetrain.blocker.setPower(0);

        // Reset encoder at closed position = 0
        drivetrain.blocker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Enable RUN_TO_POSITION mode
        drivetrain.blocker.setTargetPosition(170);
        drivetrain.blocker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivetrain.blocker.setPower(0.5);
        sleep(500);
        // -------------------------------
        // MAIN LOOP
        // -------------------------------
        while (opModeIsActive()) {

            // -------------------------------
            // DRIVING
            // -------------------------------
            double y  = gamepad1.left_stick_y;     // forward/back
            double x  = -gamepad1.left_stick_x;    // strafe
            double rx = -gamepad1.right_stick_x;   // rotate

            drivetrain.drive(y, x, rx, isFieldCentric);


            // -------------------------------
            // FIELD-CENTRIC TOGGLE (A)
            // -------------------------------
            if(controllerIsRed) {
                gamepad1.setLedColor(255,0,0,30000);
            } else gamepad1.setLedColor(0,0,255,30000);

            controllerIsRed = gamepad1.psWasPressed() && !controllerIsRed;

            // -------------------------------
            // INTAKE CONTROL
            // -------------------------------
            if (gamepad1.left_trigger > 0.5 || gamepad1.right_bumper ) {
                drivetrain.intakeOut();
            } else {
                drivetrain.intakeStop();
            }

            // -------------------------------
            // FLYWHEEL + SHOOTER
            // -------------------------------
            if (gamepad1.yWasPressed()) {
                drivetrain.setFlywheelRPM(shootingPower);

            } else if (gamepad1.aWasPressed()) {
                drivetrain.setFlywheelRPM(0);
            }
            if (gamepad1.dpad_left) {
                drivetrain.openShoot();
            } else if (gamepad1.dpad_right) {
                drivetrain.closeShoot();
            } else {
                drivetrain.stopShoot();
            }


            // -------------------------------
            // FEEDER CONTROL
            // -------------------------------
            if (gamepad1.dpad_up ) {
                drivetrain.setFeederPower(1);
            } else if (gamepad1.dpad_down) {
                drivetrain.setFeederPower(-1);
            } else {
                drivetrain.setFeederPower(0);
            }

            if (gamepad1.squareWasPressed()) {
                shootingPower+= 20;
            } else if (gamepad1.circleWasPressed()) {
                shootingPower -= 20;
            }



            // -------------------------------
            // TELEMETRY
            // -------------------------------
            telemetry.addData("Drive Mode", isFieldCentric ? "Field" : "Robot");
            telemetry.addData("Blocker Pos", drivetrain.blocker.getCurrentPosition());
            telemetry.addData("intake Power", drivetrain.intake.getPower());
            telemetry.addData("feeder Power", drivetrain.feeder.getPower());
            telemetry.addData("shoot power", shootingPower);
            telemetry.update();
        }
    }
}
