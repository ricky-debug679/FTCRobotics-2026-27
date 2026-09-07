package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

    @TeleOp(name="Turret Test: Limelight3A Stabilize", group="Test")
    public class TurretLimelight3AStabilize extends OpMode {

        // ===== TUNING =====
        private static final double kP_TX = 0.02;      // start 0.01–0.03
        private static final double kF_YAW = 0.002;    // start 0; later try ~0.002–0.010
        private static final double DEAD_BAND_DEG = 0.6;
        private static final double MAX_PWR = 0.35;

        // Flip this if turret moves the wrong way
        private static final double SIGN = -1.0;

        private DcMotorEx turret;
        private IMU imu;
        private Limelight3A limelight;

        @Override
        public void init() {
            turret = hardwareMap.get(DcMotorEx.class, "turret");
            turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters imuParams = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
            imu.initialize(imuParams);
            imu.resetYaw();

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);  // must be called before start() :contentReference[oaicite:1]{index=1}
            limelight.start();             // begins polling :contentReference[oaicite:2]{index=2}

            telemetry.addLine("Turret Stabilize Test ready.");
            telemetry.addLine("Put Limelight on an AprilTag pipeline. Rotate robot by hand; turret should counter-rotate.");
            telemetry.addLine("Hold B to stop turret (kill switch).");
            telemetry.update();
        }

        @Override
        public void loop() {
            // Kill switch
            if (gamepad1.b) {
                turret.setPower(0);
                telemetry.addLine("KILL: turret power = 0");
                telemetry.update();
                return;
            }

            // 1) Get latest Limelight result
            LLResult result = limelight.getLatestResult();  // :contentReference[oaicite:3]{index=3}
            boolean hasTarget = (result != null && result.isValid()); // :contentReference[oaicite:4]{index=4}

            double tx = 0.0;
            if (hasTarget) {
                tx = result.getTx(); // horizontal offset in degrees :contentReference[oaicite:5]{index=5}
            }

            // 2) IMU yaw rate for feedforward (deg/s)
            double yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            // 3) Control: P on tx + optional FF on yaw rate
            double pTerm = 0.0;
            if (hasTarget && Math.abs(tx) > DEAD_BAND_DEG) {
                pTerm = SIGN * (kP_TX * tx);
            }

            double ffTerm = SIGN * ((-yawRate) * kF_YAW);

            double power = pTerm + ffTerm;
            power = Range.clip(power, -MAX_PWR, MAX_PWR);

            // If no target, safest behavior is stop (for a motor test)
            if (!hasTarget) power = 0.0;

            turret.setPower(power);

            telemetry.addData("HasTarget", hasTarget);
            telemetry.addData("tx (deg)", "%.2f", tx);
            telemetry.addData("yawRate (deg/s)", "%.2f", yawRate);
            telemetry.addData("pTerm", "%.3f", pTerm);
            telemetry.addData("ffTerm", "%.3f", ffTerm);
            telemetry.addData("turretPower", "%.3f", power);
            telemetry.update();
        }

        @Override
        public void stop() {
            turret.setPower(0);
            limelight.stop(); // stops polling :contentReference[oaicite:6]{index=6}
        }
    }


