package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Decode - aura-Tele-Op")
public class TeleOp extends LinearOpMode {

    private Drivetrain drivetrain;
    private double shootingPower = -3420; // -.67
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

        // -------------------------------
        // MAIN LOOP
        // -------------------------------
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


            if (gamepad1.touchpadWasPressed()) {
                shootingPower = CF.FarRPM;
            } else if(gamepad1.psWasPressed()) {
                shootingPower = CF.CloseRPM;
            }

            // -------------------------------
            // INTAKE CONTROL
            // -------------------------------
            if (gamepad1.left_trigger > 0.5 ) {
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
            // FLYWHEEL + SHOOTER
            // -------------------------------
            if (gamepad1.yWasPressed()) {
                drivetrain.setFlywheelRPM(shootingPower);

            } else if (gamepad1.aWasPressed()) {
                drivetrain.setFlywheelRPM(0);
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
