package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret control:
 *  - Feedforward cancels robot rotation immediately using IMU yaw rate (deg/s)
 *  - Feedback recenters using AprilTag bearing (deg)
 */
public class CameraTurretController {

    private final Servo panServo;

    // ===== TUNING (more immediate) =====
    private static final double kP_BEARING = 0.020;   // trims to center
    private static final double kF_YAWRATE = 0.0040;  // counter-rotate faster (key for “instant”)

    private static final double DEAD_BAND_DEG = 0.8;
    private static final double MAX_STEP = 0.18;      // allow faster servo changes

    private static final double MIN_POS = 0.15;
    private static final double MAX_POS = 0.85;

    private final ElapsedTime tagLostTimer = new ElapsedTime();
    private static final double TAG_HOLD_TIME = 0.30;

    private double currentPos;

    // If turret moves the wrong direction while spinning robot, flip this to -1.0
    private static final double SIGN = 1.0;

    public CameraTurretController(Servo panServo) {
        this.panServo = panServo;
        this.currentPos = panServo.getPosition();
        tagLostTimer.reset();
    }

    public void update(boolean tagVisible, double bearingDeg, double robotYawRateDegPerSec) {

        // Feedforward: cancel robot rotation NOW
        double ff = SIGN * (-robotYawRateDegPerSec) * kF_YAWRATE;

        // Feedback: center using bearing
        double fb = 0.0;
        if (tagVisible) {
            tagLostTimer.reset();
            if (Math.abs(bearingDeg) >= DEAD_BAND_DEG) {
                fb = SIGN * (bearingDeg * kP_BEARING);
            }
        } else {
            // if tag recently lost, keep relying on ff (still helps a lot)
            if (tagLostTimer.seconds() > TAG_HOLD_TIME) {
                // nothing extra needed
            }
        }

        double step = ff + fb;
        step = Range.clip(step, -MAX_STEP, MAX_STEP);

        currentPos = Range.clip(currentPos + step, MIN_POS, MAX_POS);
        panServo.setPosition(currentPos);
    }

    public double getPosition() {
        return currentPos;
    }
}
