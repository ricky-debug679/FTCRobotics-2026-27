package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "FINAL EVERYTHING: Drive + Intake + Transfer + Gate + Shooter + Auto Aim", group = "TeleOp")
public class FinalEverythingTeleOp extends LinearOpMode {

    // ================= TAG IDS =================
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    // ================= DISTANCE SOLVER =================
    private static final double X_MIN_IN = 20.0;
    private static final double X_MAX_IN = 170.0;

    // ================= TURRET TUNING =================
    private static final double kP_TX = 0.02;
    private static final double kF_YAW = 0.002;
    private static final double DEAD_BAND = 0.6;
    private static final double MAX_TURRET_PWR = 0.4;
    private static final double MIN_TURRET_PWR = 0.08;
    private static final double TURRET_SIGN = -1.0;

    // ================= GATE POSITIONS =================
    private static final double GATE_CLOSED = 0.15;
    private static final double GATE_OPEN   = 0.65;

    // ================= HARDWARE =================
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor intakeMotor;
    private CRServo transferServo;
    private Servo gateServo;
    private DcMotor shooter1, shooter2;
    private DcMotorEx turret;

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {

        // ---------- Drivetrain ----------
        leftFront  = hardwareMap.get(DcMotor.class, "left front");
        leftBack   = hardwareMap.get(DcMotor.class, "left back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        rightBack  = hardwareMap.get(DcMotor.class, "right back");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        // ---------- Intake ----------
        intakeMotor = hardwareMap.get(DcMotor.class, "intake motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------- Transfer (CR Servo – continuous) ----------
        transferServo = hardwareMap.get(CRServo.class, "transferServo");

        // ---------- Gate Servo ----------
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(GATE_CLOSED);

        // ---------- Shooters ----------
        shooter1 = hardwareMap.get(DcMotor.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "Shooter2");

        // INVERTED SHOOTER DIRECTIONS HERE
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------- Turret ----------
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // ---------- IMU ----------
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        // ---------- Limelight ----------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("READY");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ================= DRIVE =================
            double drive = -gamepad1.right_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double strafe =  gamepad1.left_stick_x;

            double lf = drive + strafe + turn;
            double lb = drive - strafe + turn;
            double rf = drive - strafe - turn;
            double rb = drive + strafe - turn;

            double max = Math.max(1.0,
                    Math.max(Math.abs(lf),
                            Math.max(Math.abs(lb),
                                    Math.max(Math.abs(rf), Math.abs(rb)))));

            leftFront.setPower(lf / max);
            leftBack.setPower(lb / max);
            rightFront.setPower(rf / max);
            rightBack.setPower(rb / max);

            // ================= INTAKE (L2) =================
            double intakePower = gamepad1.left_trigger;
            intakeMotor.setPower(intakePower);

            // ================= TRANSFER (ALWAYS ON) =================
            transferServo.setPower(1.0);

            // ================= GATE (R1 = OPEN) =================
            if (gamepad1.right_bumper) {
                gateServo.setPosition(GATE_OPEN);
            } else {
                gateServo.setPosition(GATE_CLOSED);
            }

            // ================= LIMELIGHT =================
            boolean seesGoal = false;
            double tx = 0.0;
            double area = 0.0;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null) {
                    for (LLResultTypes.FiducialResult f : tags) {
                        if (f.getFiducialId() == BLUE_GOAL_ID ||
                                f.getFiducialId() == RED_GOAL_ID) {
                            seesGoal = true;
                            tx = f.getTargetXDegrees();
                            area = f.getTargetArea();
                            break;
                        }
                    }
                }
            }

            // ================= DISTANCE (AREA ONLY) =================
            double distanceIn = (area > 0) ? estimateDistanceFromAreaOnly(area) : -1;

            // ================= AUTO SHOOTER =================
            double shooterPower = gamepad1.right_trigger;

            if (distanceIn > 0) {
                shooterPower = shooterFormula(distanceIn);
            }

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            // ================= TURRET LOCK =================
            double turretPower = 0.0;

            if (!gamepad1.b && seesGoal) {
                double pTerm = Math.abs(tx) > DEAD_BAND ? TURRET_SIGN * kP_TX * tx : 0.0;
                double yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
                double ffTerm = TURRET_SIGN * (-yawRate * kF_YAW);
                turretPower = Range.clip(pTerm + ffTerm, -MAX_TURRET_PWR, MAX_TURRET_PWR);

                if (Math.abs(turretPower) > 0 && Math.abs(turretPower) < MIN_TURRET_PWR) {
                    turretPower = Math.copySign(MIN_TURRET_PWR, turretPower);
                }
            }

            turret.setPower(turretPower);

            // ================= TELEMETRY =================
            telemetry.addData("Goal", seesGoal);
            telemetry.addData("tx", "%.2f", tx);
            telemetry.addData("Distance (in)", "%.1f", distanceIn);
            telemetry.addData("Shooter", "%.2f", shooterPower);
            telemetry.addData("Turret", "%.2f", turretPower);
            telemetry.update();
        }

        limelight.stop();
    }

    // ================= DISTANCE SOLVER =================
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

    // ================= SHOOTER FORMULA =================
    private double shooterFormula(double d) {
        double p = 0.000015 * d * d + 0.0025 * d + 0.55;
        return Range.clip(p, 0.55, 1.0);
    }
}