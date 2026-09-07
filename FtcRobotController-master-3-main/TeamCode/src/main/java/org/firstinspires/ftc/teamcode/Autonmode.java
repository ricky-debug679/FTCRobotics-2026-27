package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Autonomous(name = "Auto: Forward → Detect → Shoot → Reverse", group = "Auto")
public class Autonmode extends LinearOpMode {

    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    private static final double kP_TX = 0.02;
    private static final double kF_YAW = 0.002;
    private static final double DEAD_BAND = 0.6;
    private static final double MAX_TURRET_PWR = 0.4;
    private static final double MIN_TURRET_PWR = 0.08;

    private static final double X_MIN_IN = 20.0;
    private static final double X_MAX_IN = 170.0;

    DcMotor lf, lb, rf, rb;
    DcMotor intake;
    CRServo transfer;
    DcMotor shooter1, shooter2;
    DcMotorEx turret;
    Limelight3A limelight;
    IMU imu;

    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotor.class, "left front");
        lb = hardwareMap.get(DcMotor.class, "left back");
        rf = hardwareMap.get(DcMotor.class, "right front");
        rb = hardwareMap.get(DcMotor.class, "right back");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake motor");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer = hardwareMap.get(CRServo.class, "transferServo");

        shooter1 = hardwareMap.get(DcMotor.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "Shooter2");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();

        /* -------- STEP 1: DRIVE FORWARD -------- */
        setDrive(0.4);
        sleep(3000);
        stopDrive();

        /* -------- STEP 2: SEARCH + SHOOT -------- */
        long shootStart = -1;

        while (opModeIsActive()) {

            boolean seesTag = false;
            double tx = 0;
            double distance = -1;

            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
                    if (f.getFiducialId() == BLUE_GOAL_ID || f.getFiducialId() == RED_GOAL_ID) {
                        seesTag = true;
                        tx = f.getTargetXDegrees();
                        distance = estimateDistanceFromAreaOnly(f.getTargetArea());
                        break;
                    }
                }
            }

            /* Intake / Transfer logic */
            if (!seesTag) {
                transfer.setPower(1.0);
                intake.setPower(0.4);
            } else {
                if (shootStart < 0) shootStart = System.currentTimeMillis();

                transfer.setPower(-1.0);
                intake.setPower(1.0);

                double turretPower = 0;
                if (Math.abs(tx) > DEAD_BAND) {
                    double p = kP_TX * tx;
                    double ff = -imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate * kF_YAW;
                    turretPower = Range.clip(p + ff, -MAX_TURRET_PWR, MAX_TURRET_PWR);
                    if (Math.abs(turretPower) < MIN_TURRET_PWR)
                        turretPower = Math.copySign(MIN_TURRET_PWR, turretPower);
                }
                turret.setPower(turretPower);

                double shooterPower = distance > 0 ? integratedShooterFormula(distance) : 0;
                shooter1.setPower(shooterPower);
                shooter2.setPower(shooterPower);

                if (System.currentTimeMillis() - shootStart > 5000) break;
            }
        }

        /* -------- STEP 3: DRIVE BACK -------- */
        setDrive(-0.4);
        sleep(3000);
        stopAll();
        limelight.stop();
    }

    /* ---------- Helpers ---------- */

    private void setDrive(double p) {
        lf.setPower(p);
        lb.setPower(p);
        rf.setPower(p);
        rb.setPower(p);
    }

    private void stopDrive() {
        setDrive(0);
    }

    private void stopAll() {
        stopDrive();
        intake.setPower(0);
        transfer.setPower(0);
        shooter1.setPower(0);
        shooter2.setPower(0);
        turret.setPower(0);
    }

    private double integratedShooterFormula(double x) {
        double p =
                0.95 * (
                        0.00000122124 * Math.pow(x, 4)
                                - 0.000320651  * Math.pow(x, 3)
                                + 0.0309334    * Math.pow(x, 2)
                                - 1.29708      * x
                                + 20.51573
                );
        return Range.clip(p, 0, 1);
    }

    private double estimateDistanceFromAreaOnly(double area) {
        double bestX = X_MIN_IN;
        double bestErr = Double.MAX_VALUE;
        for (int i = 0; i <= 300; i++) {
            double x = X_MIN_IN + (X_MAX_IN - X_MIN_IN) * i / 300.0;
            double err = Math.pow(f_area(x) - area, 2);
            if (err < bestErr) {
                bestErr = err;
                bestX = x;
            }
        }
        return bestX;
    }

    private double f_area(double x) {
        return (1.44509e-9) * Math.pow(x, 4)
                - (5.76194e-7) * Math.pow(x, 3)
                + (8.48292e-5) * Math.pow(x, 2)
                - (5.63797e-3) * x
                + 0.152368;
    }
}
