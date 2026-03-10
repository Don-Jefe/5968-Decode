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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.NotOpModes.PoseStorage;

@Configurable
@TeleOp(name = "Chat-cooked FINAL")
public class ChatTele extends OpMode {

    /* ================= FIELD ================= */
    public static final double FIELD_SIZE = 144.0;

    /* ================= AIMBOT ================= */
    public static double TURN_kP = 2.0;
    public static double MAX_TURN_POWER = 0.4;
    public static double AIM_DELAY_SEC = 0.10;

    /* ================= SHOT MODEL ================= */
    public static double SHOT_SPEED_IPS = 300.0;

    /* ================= RPM CONTROL ================= */
    public static double RPM_ACCEL = 4500;
    public static double RPM_DECEL = 9000;
    public static double RPM_TOLERANCE = 75;
    public static double ALIGN_TOLERANCE_DEG = 2.7;

    private double targetRPM = 0;
    private double rampedRPM = 0;
    private double lastLoopTime = 0;

    /* ================= SMOOTHING ================= */
    private double filteredDistance = 0;

    /* ================= HARDWARE ================= */
    private Follower follower;
    private Drivetrain drivetrain;
    private TelemetryManager telemetryM;

    /* ================= TARGET ================= */
    private double targetX = 8;
    private double targetY = FIELD_SIZE - 8;

    /* ================= STATE ================= */
    private ElapsedTime aimTimer = new ElapsedTime();
    private boolean wasAimbotActive = false;

    /* ================= HELPERS ================= */

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double getDistance(Pose pose) {
        return Math.hypot(targetX - pose.getX(), targetY - pose.getY());
    }

    private double getFlywheelRPMForDistance(double d) {
        if (d < 140)
            return 0.0701569 * d * d - 7.65244 * d + 2808.8812;
        return 3100;
    }

    private double getActualRPM() {
        return (drivetrain.flywheel.getVelocity() / 28.0) * 60.0;
    }

    private boolean isFlywheelReady() {
        return Math.abs(getActualRPM() - targetRPM) < RPM_TOLERANCE;
    }

    /* ================= LEAD COMPENSATION ================= */

    private double getTimeOfFlight(double distance) {
        return distance / SHOT_SPEED_IPS;
    }

    private double[] getLeadAdjustedTarget(Pose pose) {

        double rawDistance = getDistance(pose);
        double tof = getTimeOfFlight(rawDistance);

        Vector vel = follower.getVelocity();
        double vx = vel.getXComponent();
        double vy = vel.getYComponent();

        // Shift basket opposite of robot motion
        double leadX = targetX - vx * tof;
        double leadY = targetY - vy * tof;

        return new double[]{leadX, leadY};
    }

    private boolean isAligned(Pose pose) {
        double[] leadTarget = getLeadAdjustedTarget(pose);

        double dx = leadTarget[0] - pose.getX();
        double dy = leadTarget[1] - pose.getY();

        double desired = Math.atan2(dy, dx);
        double error = normalizeAngle(desired - pose.getHeading());

        return Math.abs(Math.toDegrees(error)) < ALIGN_TOLERANCE_DEG;
    }

    /* ================= VELOCITY DISTANCE PREDICTION ================= */

    private double getPredictedDistance(Pose pose) {

        double rawDistance = getDistance(pose);

        Vector vel = follower.getVelocity();
        double vx = vel.getXComponent();
        double vy = vel.getYComponent();

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();

        double distance = Math.hypot(dx, dy);
        double dirX = dx / distance;
        double dirY = dy / distance;

        double closingVelocity = vx * dirX + vy * dirY;

        // Predict 0.2 seconds ahead
        return rawDistance - closingVelocity * 0.2;
    }

    /* ================= TURN WITH LEAD ================= */

    private double getTurnToTarget(Pose pose) {

        double[] leadTarget = getLeadAdjustedTarget(pose);

        double dx = leadTarget[0] - pose.getX();
        double dy = leadTarget[1] - pose.getY();

        double desired = Math.atan2(dy, dx);
        double error = normalizeAngle(desired - pose.getHeading());

        return clamp(error * TURN_kP, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    /* ================= RPM RAMP ================= */

    private void updateRamp(double dt, double predictedDistance, double rawDistance) {

        targetRPM = getFlywheelRPMForDistance(predictedDistance);

        double distanceDelta = predictedDistance - rawDistance;

        // Moving closer → ramp down harder
        if (distanceDelta < 0) {
            rampedRPM -= RPM_DECEL * 1.5 * dt;
        }
        // Moving away → ramp up sooner
        else if (distanceDelta > 0) {
            rampedRPM += RPM_ACCEL * 1.3 * dt;
        }

        if (rampedRPM < targetRPM) {
            rampedRPM += RPM_ACCEL * dt;
        } else {
            rampedRPM -= RPM_DECEL * dt;
        }

        rampedRPM = clamp(rampedRPM, 0, 3500);
    }

    /* ================= INIT ================= */

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        lastLoopTime = getRuntime();
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        Pose pose = follower.getPose();

        boolean aimbot = gamepad1.right_trigger > 0.7;

        if (aimbot && !wasAimbotActive) aimTimer.reset();
        wasAimbotActive = aimbot;

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = aimbot ? getTurnToTarget(pose) : -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, turn, true);

        double rawDistance = getDistance(pose);

        filteredDistance = 0.8 * filteredDistance + 0.2 * rawDistance;

        double predictedDistance = getPredictedDistance(pose);

        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;

        updateRamp(dt, predictedDistance, rawDistance);

        drivetrain.NewSetFlywheelRPM(-rampedRPM, CF.PPP, 0, 0, CF.FFF);

        if (aimbot &&
                aimTimer.seconds() >= AIM_DELAY_SEC &&
                isFlywheelReady() &&
                isAligned(pose)) {

            drivetrain.setIntakePower(-1);
            drivetrain.setFeederPower(1);
        } else {
            drivetrain.setIntakePower(0);
            drivetrain.setFeederPower(0);
        }

        telemetryM.debug("Raw Distance", rawDistance);
        telemetryM.debug("Predicted Distance", predictedDistance);
        telemetryM.debug("Target RPM", targetRPM);
        telemetryM.debug("Ramped RPM", rampedRPM);
        telemetryM.debug("Actual RPM", getActualRPM());
        telemetryM.debug("Flywheel Ready", isFlywheelReady());
        telemetryM.debug("Aligned (Lead)", isAligned(pose));
    }
}
