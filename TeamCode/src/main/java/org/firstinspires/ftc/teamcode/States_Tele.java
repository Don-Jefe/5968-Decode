package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NotOpModes.CF;
import org.firstinspires.ftc.teamcode.NotOpModes.Drivetrain;
import org.firstinspires.ftc.teamcode.NotOpModes.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Goop GOOP")
public class States_Tele extends OpMode {

    /* ================= FIELD ================= */

    public static final double FIELD_SIZE = 144.0;

    public static final double BLUE_RESET_X = 24.24;
    public static final double BLUE_RESET_Y = 127.03;
    public static final double BLUE_RESET_HEADING = Math.toRadians(130);

    public static final double RED_RESET_X = 125.0;
    public static final double RED_RESET_Y = 130.0;
    public static final double RED_RESET_HEADING = Math.toRadians(37);

    public static final double RED_BACK_RESET_X = 136;
    public static final double RED_BACK_RESET_Y = 8;

    public static final double BLUE_BACK_RESET_X = 8;
    public static final double BLUE_BACK_RESET_Y = 8;

    public static final double DEFAULT_RESET_HEADING = Math.toRadians(90);

    /* ================= AIM ================= */

    public static double TURN_kP = 2;
    public static double MAX_TURN_POWER = 0.40;
    public static double AIM_DELAY_SEC = 0.10;

    /* ================= LEAD SHOT ================= */

    public static double SHOT_SPEED_IPS = 230;
    private boolean leadShotEnabled = false;

    /* ================= FLYWHEEL ================= */

    public static double RPM_ACCEL_NORMAL = 9000;
    public static double RPM_ACCEL_RECOVERY = 20000;
    public static double RPM_DECEL = 3000;
    public static double RECOVERY_THRESHOLD = 150;

    public static double FEEDER_POWER = 1.0;
    public static double RPM_TOLERANCE = 120;
    public static double RPM_UNLOCK = 200;

    private double targetRPM = 0;
    private double rampedRPM = 0;
    private double lastLoopTime = 0;

    private boolean flywheelReady = false;

    /* ================= HARDWARE ================= */

    private Follower follower;
    private Drivetrain drivetrain;
    private TelemetryManager telemetryM;

    /* ================= TARGET ================= */

    private double targetX;
    private double targetY;
    private boolean isBlueAlliance = true;

    /* ================= AIM STATE ================= */

    private final ElapsedTime aimTimer = new ElapsedTime();
    private boolean wasAimbotActive = false;

    /* ================================================= */

    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        updateAllianceTarget();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        lastLoopTime = getRuntime();
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {

        updateAllianceLED();

        follower.update();
        telemetryM.update();

        Pose pose = follower.getPose();

        handleAllianceToggle();
        handlePoseReset();
        handleLeadToggle();

        boolean aimbotActive = gamepad1.right_trigger > 0.7;

        handleDrive(pose, aimbotActive);

        updateFlywheelControl(pose);

        handleShooting(aimbotActive);

        drivetrain.updateBlocker(gamepad1);
        drivetrain.updateIntake(gamepad1.left_trigger);

        sendTelemetry(pose);
    }

    /* ================= DRIVE ================= */

    private void handleDrive(Pose pose, boolean aimbotActive) {

        if (aimbotActive && !wasAimbotActive) {
            aimTimer.reset();
        }

        wasAimbotActive = aimbotActive;

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;

        double turn = aimbotActive
                ? getTurnToTarget(pose)
                : -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    /* ================= FLYWHEEL ================= */

    private void updateFlywheelControl(Pose pose) {

        double distance = getDistanceToBasket(pose);
        targetRPM = getFlywheelRPMForDistance(distance);

        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;

        updateFlywheelRamp(dt);

        drivetrain.setDualFlywheelRPM(
                -rampedRPM,
                CF.PPP,
                0,
                0,
                CF.FFF
        );
    }

    private void updateFlywheelRamp(double dt) {

        double error = targetRPM - rampedRPM;

        if (error > 0) {

            double accel = (error > RECOVERY_THRESHOLD)
                    ? RPM_ACCEL_RECOVERY
                    : RPM_ACCEL_NORMAL;

            rampedRPM += accel * dt;

            if (rampedRPM > targetRPM)
                rampedRPM = targetRPM;

        } else {

            rampedRPM -= RPM_DECEL * dt;

            if (rampedRPM < targetRPM)
                rampedRPM = targetRPM;
        }
    }

    /* ================= SHOOTING ================= */

    private void handleShooting(boolean aimbotActive) {

        boolean shooting = aimbotActive && aimTimer.seconds() >= AIM_DELAY_SEC;

        if (shooting) {

            drivetrain.setIntakePower(-1);

            double actualRPM = getActualFlywheelRPM();

            // Compare to ramped RPM (NOT target)
            double rpmError = Math.abs(actualRPM - rampedRPM);

            if (!flywheelReady && rpmError <= RPM_TOLERANCE)
                flywheelReady = true;

            if (flywheelReady && rpmError > RPM_UNLOCK)
                flywheelReady = false;

            drivetrain.setFeederPower(
                    flywheelReady ? FEEDER_POWER : 0
            );

        } else {

            flywheelReady = false;

            if (gamepad1.left_trigger < 0.5) {
                drivetrain.setIntakePower(0);
                drivetrain.setFeederPower(0);
            }

            drivetrain.updateFeeder(gamepad1);
        }
    }

    /* ================= AIM ================= */

    private double getTurnToTarget(Pose pose) {

        double basketX = targetX;
        double basketY = targetY;
        double tof = 0;

        if (leadShotEnabled) {

            double distance = getDistanceToBasket(pose);
            tof = distance / SHOT_SPEED_IPS;

            Vector vel = follower.getVelocity();

            basketX -= vel.getXComponent() * tof;
            basketY -= vel.getYComponent() * tof;
        }

        double dx = basketX - pose.getX();
        double dy = basketY - pose.getY();

        double desiredHeading = Math.atan2(dy, dx);
        double error = normalizeAngle(desiredHeading - pose.getHeading());

        return clamp(error * TURN_kP, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    /* ================= HELPERS ================= */

    private void handleAllianceToggle() {
        if (gamepad1.psWasPressed()) {
            isBlueAlliance = !isBlueAlliance;
            updateAllianceTarget();
        }
    }

    private void handlePoseReset() {

        if (gamepad1.dpadUpWasPressed()) {

            follower.setPose(isBlueAlliance
                    ? new Pose(BLUE_RESET_X, BLUE_RESET_Y, BLUE_RESET_HEADING)
                    : new Pose(RED_RESET_X, RED_RESET_Y, RED_RESET_HEADING));

        } else if (gamepad1.dpadDownWasPressed()) {

            follower.setPose(isBlueAlliance
                    ? new Pose(BLUE_BACK_RESET_X, BLUE_BACK_RESET_Y, DEFAULT_RESET_HEADING)
                    : new Pose(RED_BACK_RESET_X, RED_BACK_RESET_Y, DEFAULT_RESET_HEADING));
        }
    }

    private void handleLeadToggle() {
        if (gamepad1.squareWasPressed())
            leadShotEnabled = !leadShotEnabled;
    }

    private void updateAllianceTarget() {

        if (isBlueAlliance) {
            targetX = 8;
            targetY = FIELD_SIZE - 8;
        } else {
            targetX = FIELD_SIZE - 8;
            targetY = FIELD_SIZE - 8;
        }
    }

    private void updateAllianceLED() {

        if (isBlueAlliance)
            gamepad1.setLedColor(0,0,255,1000);
        else
            gamepad1.setLedColor(255,0,0,1000);
    }

    private double getDistanceToBasket(Pose pose) {
        return Math.hypot(targetX - pose.getX(), targetY - pose.getY());
    }

    private double getFlywheelRPMForDistance(double distance) {

        if (distance < 140)
            return 0.0701569 * distance * distance
                    - 7.65244 * distance
                    + 2808.8812;

        return 3100;
    }

    private double getActualFlywheelRPM() {

        double ticksPerSecond = drivetrain.flywheel.getVelocity();
        return (ticksPerSecond / 28.0) * 60.0;
    }

    private double normalizeAngle(double angle) {

        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;

        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /* ================= TELEMETRY ================= */

    private void sendTelemetry(Pose pose) {

        telemetryM.debug("Alliance", isBlueAlliance ? "BLUE" : "RED");
        telemetryM.debug("Pose", pose);

        telemetryM.debug("Target RPM", targetRPM);
        telemetryM.debug("Ramped RPM", rampedRPM);
        telemetryM.debug("Actual RPM", getActualFlywheelRPM());

        telemetryM.debug("Flywheel Ready", flywheelReady);
        telemetryM.debug("Lead Shot", leadShotEnabled ? "ON" : "OFF");
    }
}