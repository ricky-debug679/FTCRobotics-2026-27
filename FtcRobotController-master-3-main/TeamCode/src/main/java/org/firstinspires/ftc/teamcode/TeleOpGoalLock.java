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

@TeleOp(name = "FINAL FIXED: Drive + RPM + LL Turret + Gate", group = "TeleOp")
public class TeleOpGoalLock extends LinearOpMode {

    // ================= TAG IDS =================
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    // ================= TURRET TUNING =================
    private static final double kP_TX = 0.02;
    private static final double kF_YAW = 0.002;
    private static final double DEAD_BAND = 0.6;
    private static final double MAX_TURRET_PWR = 0.4;
    private static final double MIN_TURRET_PWR = 0.08;
    private static final double TURRET_SIGN = -1.0;

    // ================= GATE POSITIONS =================
    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_OPEN   = 1.0;

    // ================= HARDWARE =================
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intakeMotor;
    private CRServo transferServo;
    private Servo gateServo;
    private DcMotorEx shooter1, shooter2;
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
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---------- Transfer + Gate ----------
        transferServo = hardwareMap.get(CRServo.class, "transferServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(GATE_CLOSED);

        // ---------- Shooters ----------
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---------- Turret ----------
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        limelight.pipelineSwitch(0);   // must match AprilTag pipeline
        limelight.start();


        telemetry.addLine("READY: All systems locked");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= DRIVE =================
            double drive = -gamepad1.right_stick_y;
            double turn  =  gamepad1.right_stick_x;

            leftFront.setPower(Range.clip(drive + turn, -1, 1));
            leftBack.setPower(Range.clip(drive + turn, -1, 1));
            rightFront.setPower(Range.clip(drive - turn, -1, 1));
            rightBack.setPower(Range.clip(drive - turn, -1, 1));

            // ================= INTAKE =================
            intakeMotor.setPower(gamepad1.left_trigger);

            // ================= SHOOTERS =================
            double shooterPower = gamepad1.right_trigger;
            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            // ================= TRANSFER + GATE =================
            if (gamepad1.cross) {
                gateServo.setPosition(GATE_OPEN);
                transferServo.setPower(1.0); // continuous inward feed
            } else {
                gateServo.setPosition(GATE_CLOSED);
                transferServo.setPower(-1.0);
            }

            // ================= RPM =================
            double intakeRPM  = intakeMotor.getVelocity() * 60.0 /
                    intakeMotor.getMotorType().getTicksPerRev();
            double shooter1RPM = shooter1.getVelocity() * 60.0 /
                    shooter1.getMotorType().getTicksPerRev();
            double shooter2RPM = shooter2.getVelocity() * 60.0 /
                    shooter2.getMotorType().getTicksPerRev();

            // ================= LIMELIGHT =================
            boolean seesGoal = false;
            double tx = 0.0;

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null) {
                    for (LLResultTypes.FiducialResult f : tags) {
                        if (f.getFiducialId() == BLUE_GOAL_ID ||
                                f.getFiducialId() == RED_GOAL_ID) {
                            seesGoal = true;
                            tx = f.getTargetXDegrees();
                            break;
                        }
                    }
                }
            }

            // ================= TURRET =================
            double turretPower = 0.0;

            if (!gamepad1.b && seesGoal) {

                double pTerm = 0.0;
                if (Math.abs(tx) > DEAD_BAND) {
                    pTerm = TURRET_SIGN * kP_TX * tx;
                }

                double yawRate =
                        imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

                double ffTerm = TURRET_SIGN * (-yawRate * kF_YAW);

                turretPower = Range.clip(
                        pTerm + ffTerm,
                        -MAX_TURRET_PWR,
                        MAX_TURRET_PWR
                );

                if (Math.abs(turretPower) > 0 &&
                        Math.abs(turretPower) < MIN_TURRET_PWR) {
                    turretPower = Math.copySign(MIN_TURRET_PWR, turretPower);
                }
            }

            turret.setPower(turretPower);

            // ================= TELEMETRY =================
            telemetry.addData("Intake RPM", "%.0f", intakeRPM);
            telemetry.addData("Shooter1 RPM", "%.0f", shooter1RPM);
            telemetry.addData("Shooter2 RPM", "%.0f", shooter2RPM);
            telemetry.addData("Sees Goal", seesGoal);
            telemetry.addData("Turret Power", "%.2f", turretPower);
            telemetry.update();
        }

        limelight.stop();
    }
}
