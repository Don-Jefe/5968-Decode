package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Do not Run!!!!!")
public class HeySisters extends LinearOpMode {

    private Drivetrain drivetrain;
    private double shootingPower = -3420; // -.67

    private boolean isFieldCentric = false;
    private boolean controllerIsRed;

    // RGB breathing state
    private double ledTime = 0;
    private final double BREATH_SPEED = 0.1; // higher = faster breathing

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
        drivetrain.blocker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivetrain.blocker.setPower(-1);
        sleep(500);
        drivetrain.blocker.setPower(0);

        drivetrain.blocker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivetrain.blocker.setTargetPosition(170);
        drivetrain.blocker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivetrain.blocker.setPower(0.5);
        sleep(500);

        telemetry.addLine("Blocker calibrated.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // -------------------------------
        // MAIN LOOP
        // -------------------------------
        while (opModeIsActive()) {

            // -------------------------------
            // DRIVING
            // -------------------------------
            double y  = gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            drivetrain.drive(y, x, rx, isFieldCentric);


            // -------------------------------
            // SHOOTER PRESETS
            // -------------------------------
            if (gamepad1.touchpadWasPressed()) {
                shootingPower = -4540;
            } else if (gamepad1.psWasPressed()) {
                shootingPower = -3380;
            }

            // -------------------------------
            // INTAKE CONTROL
            // -------------------------------
            if (gamepad1.left_trigger > 0.5) {
                drivetrain.intakeOut();
            } else {
                drivetrain.intakeStop();
            }

            if (gamepad1.right_bumper) {
                drivetrain.closeShoot();
            } else {
                drivetrain.stopShoot();
            }

            if (gamepad1.left_bumper) {
                drivetrain.openShoot();
            } else {
                drivetrain.stopShoot();
            }

            // -------------------------------
            // FLYWHEEL
            // -------------------------------
            if (gamepad1.yWasPressed()) {
                drivetrain.setFlywheelRPM(shootingPower);
            } else if (gamepad1.aWasPressed()) {
                drivetrain.setFlywheelRPM(0);
            }

            // -------------------------------
            // FEEDER
            // -------------------------------
            if (gamepad1.dpad_up) {
                drivetrain.setFeederPower(1);
            } else if (gamepad1.dpad_down) {
                drivetrain.setFeederPower(-1);
            } else {
                drivetrain.setFeederPower(0);
            }

            if (gamepad1.squareWasPressed()) {
                shootingPower += 20;
            } else if (gamepad1.circleWasPressed()) {
                shootingPower -= 20;
            }


            // ----------------------------------------------------
            // CONTROLLER RGB BREATHING EFFECT
            // ----------------------------------------------------
            ledTime += 0.04; // ~25Hz update rate

            double breath = (Math.sin(ledTime * BREATH_SPEED) + 1) / 2.0; // 0 → 1

            int r = (int)(breath * 255);
            int g = (int)((1 - breath) * 255);
            int b = 150; // soft blue always present

            // Set controller LED (PS Only)
            gamepad1.setLedColor(r, g, b, 10); // 10ms = fast refresh


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
