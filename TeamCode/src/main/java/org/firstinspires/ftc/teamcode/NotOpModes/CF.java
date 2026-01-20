package org.firstinspires.ftc.teamcode.NotOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class CF {

    // ======================================================================
    //  TELEOP VARS
    // ======================================================================
    public static int CloseRPM = -2800;
    public static int FarRPM = -3200;

    public static final double SERVO_TOP_POS = .45;
    public static final double SERVO_BOTTOM_POS = 0.76;

    public static  double intakePower = 0.6;

    // ======================================================================
    //  POSES FOR AUTON — ALL COORDINATES CENTRALIZED
    // ======================================================================

    // ---- Shooting Position 1 ----
    public static  Pose S1_START = new Pose(27.369, 131.478);
    public static  Pose S1_END   = new Pose(48.119, 95.523);
    public static  double S1_HEAD_START = Math.toRadians(143);
    public static  double S1_HEAD_END   = Math.toRadians(129);

    // ---- Pickup 1 ----
    public static  Pose P1_START = new Pose(48.119, 95.523);
    public static  Pose P1_DROP  = new Pose(48.119, 83.896);
    public static  Pose P1_END   = new Pose(16.815, 83.896);
    public static  double P1_HEAD_START = Math.toRadians(129);
    public static  double P1_HEAD_DOWN  = Math.toRadians(180);
    public static  double P1_HEAD_END   = Math.toRadians(180);

    // ---- Shooting Position 2 ----
    public static  Pose S2_START = new Pose(16.815, 83.896);
    public static  Pose S2_END   = new Pose(48.119, 95.523);
    public static  double S2_HEAD_START = Math.toRadians(180);
    public static  double S2_HEAD_END   = Math.toRadians(129);

    // ---- Pickup 2 ----
    public static  Pose P2_START = new Pose(48.119, 95.523);
    public static  Pose P2_DROP  = new Pose(47.940, 59.925);
    public static  Pose P2_END   = new Pose(23.791, 59.925);
    public static  double P2_HEAD_START = Math.toRadians(129);
    public static  double P2_HEAD_DOWN  = Math.toRadians(180);
    public static  double P2_HEAD_END   = Math.toRadians(180);

    // ---- Backup Path ----
    public static  Pose BACKUP_START = new Pose(23.791, 59.925);
    public static  Pose BACKUP_END   = new Pose(47.940, 59.925);
    public static  double BACKUP_HEAD_START = Math.toRadians(180);
    public static  double BACKUP_HEAD_END   = Math.toRadians(150);

    // ---- Shooting Position 3 ----
    public static  Pose S3_START = new Pose(47.940, 59.925);
    public static  Pose S3_END   = new Pose(48.119, 95.523);
    public static  double S3_HEAD_START = Math.toRadians(150);
    public static  double S3_HEAD_END   = Math.toRadians(129);

    // ---- Pickup 3 ----
    public static  Pose P3_START = new Pose(48.119, 95.523);
    public static  Pose P3_DROP  = new Pose(47.583, 35.061);
    public static  Pose P3_END   = new Pose(19.856, 35.240);
    public static  double P3_HEAD_START = Math.toRadians(129);
    public static  double P3_HEAD_DOWN  = Math.toRadians(180);
    public static  double P3_HEAD_END   = Math.toRadians(180);

    // ---- Shooting Position 4 ----
    public static  Pose S4_START = new Pose(19.856, 35.240);
    public static  Pose S4_END   = new Pose(48.298, 95.165);
    public static  double S4_HEAD_START = Math.toRadians(180);
    public static  double S4_HEAD_END   = Math.toRadians(129);


}
