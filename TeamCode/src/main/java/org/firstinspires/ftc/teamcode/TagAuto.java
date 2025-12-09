package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import com.qualcomm.robotcore.hardware.Gamepad;

@Autonomous(name = "B I N G O and BINGO was her name-o", group = "Examples")
public class TagAuto extends OpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor processor;

    @Override
    public void init() {
        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .enableLiveView(true)
                .build();
    }

    @Override
    public void loop() {
        telemetry.addData("Tags Detected", processor.getDetections().size());
        for (AprilTagDetection tag : processor.getDetections()) {
            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Position", "(%.1f, %.1f, %.1f)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
        }
        telemetry.update();
    }
}


