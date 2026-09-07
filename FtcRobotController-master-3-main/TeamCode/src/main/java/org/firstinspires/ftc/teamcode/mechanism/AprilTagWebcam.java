package org.firstinspires.ftc.teamcode.mechanism;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));

        // ✅ LOWER LATENCY for “immediate” tracking
        builder.setCameraResolution(new Size(320, 240));

        builder.addProcessor(aprilTagProcessor);

        // ✅ Live view can reduce performance. Turn off unless you really need it.
        builder.enableLiveView(false);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {
            telemetry.addLine("Tag not visible");
            return;
        }

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("ID %d (%s)", detectedId.id, detectedId.metadata.name));
        } else {
            telemetry.addLine(String.format("ID %d (Unknown)", detectedId.id));
        }

        telemetry.addLine(String.format(
                "Range: %.1f in | Bearing: %.1f | Yaw: %.1f",
                detectedId.ftcPose.range,
                detectedId.ftcPose.bearing,
                detectedId.ftcPose.yaw
        ));
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
