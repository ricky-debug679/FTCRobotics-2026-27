package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.reflect.Method;
import java.util.List;

@TeleOp(name = "LL: Goal + Distance + Turret Stabilized", group = "TeleOp")
public class Teleop extends LinearOpMode {

    // ================== TUNING ==================
    private static final double kP_TX = 0.02;
    private static final double kF_YAW = 0.002;
    private static final double DEAD_BAND_DEG = 0.6;
    private static final double MAX_SERVO_STEP = 0.03;
    private static final double TURRET_SIGN = -1.0;

    // Tag IDs
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID  = 24;
    private static final int LIMELIGHT_PIPELINE = 0;

    // Distance bounds (in)
    private static final double X_MIN_IN = 20.0;
    private static final double X_MAX_IN = 170.0;

    // Hardware
    private DcMotor shootermotor, intakemotor;
    private DcMotor leftfront, leftback, rightfront, rightback;
    private Servo cameraPan;

    private IMU imu;
    private Limelight3A limelight;

    private double turretPos = 0.5;

    private enum GoalMode { AUTO_MOST_CENTERED, FORCE_BLUE, FORCE_RED }
    private GoalMode goalMode = GoalMode.AUTO_MOST_CENTERED;

    @Override
    public void runOpMode() {

        shootermotor = hardwareMap.get(DcMotor.class, "shooter motor");
        intakemotor  = hardwareMap.get(DcMotor.class, "intake motor");

        leftfront  = hardwareMap.get(DcMotor.class, "left front");
        leftback   = hardwareMap.get(DcMotor.class, "left back");
        rightfront = hardwareMap.get(DcMotor.class, "right front");
        rightback  = hardwareMap.get(DcMotor.class, "right back");

        cameraPan = hardwareMap.get(Servo.class, "camera_pan");
        cameraPan.setPosition(turretPos);

        shootermotor.setDirection(DcMotor.Direction.REVERSE);
        intakemotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        telemetry.addLine("READY: Turret stabilized + goal lock");
        telemetry.update();

        waitForStart();

        boolean lastL=false,lastR=false,lastU=false;

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && !lastU) goalMode = GoalMode.AUTO_MOST_CENTERED;
            if (gamepad1.dpad_left && !lastL) goalMode = GoalMode.FORCE_BLUE;
            if (gamepad1.dpad_right && !lastR) goalMode = GoalMode.FORCE_RED;

            lastU=gamepad1.dpad_up;
            lastL=gamepad1.dpad_left;
            lastR=gamepad1.dpad_right;

            double yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            LLResult result = limelight.getLatestResult();
            boolean hasTarget = false;
            double tx = 0.0;
            double distanceIn = -1.0;
            String goalName = "NONE";

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> f = result.getFiducialResults();
                var blue = getFiducialById(f, BLUE_GOAL_TAG_ID);
                var red  = getFiducialById(f, RED_GOAL_TAG_ID);
                var chosen = chooseGoal(blue, red, goalMode);

                if (chosen != null) {
                    hasTarget = true;
                    tx = chosen.getTargetXDegrees();
                    goalName = chosen.getFiducialId()==BLUE_GOAL_TAG_ID?"BLUE":"RED";
                    double area = getSizeProxy(chosen);
                    if (area > 0) distanceIn = estimateDistanceFromAreaOnly(area);
                }
            }

            // ===== TURRET CONTROL =====
            double pTerm = 0;
            if (hasTarget && Math.abs(tx) > DEAD_BAND_DEG)
                pTerm = TURRET_SIGN * kP_TX * tx;

            double ffTerm = TURRET_SIGN * (-yawRate) * kF_YAW;
            double delta = Range.clip(pTerm + ffTerm, -MAX_SERVO_STEP, MAX_SERVO_STEP);

            if (hasTarget) {
                turretPos = Range.clip(turretPos + delta, 0.0, 1.0);
                cameraPan.setPosition(turretPos);
            }

            // ===== DRIVE =====
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            leftfront.setPower(Range.clip(drive + turn, -1, 1));
            leftback.setPower(Range.clip(drive + turn, -1, 1));
            rightfront.setPower(Range.clip(drive - turn, -1, 1));
            rightback.setPower(Range.clip(drive - turn, -1, 1));

            // Intake / Shooter
            intakemotor.setPower(gamepad1.left_trigger);
            shootermotor.setPower(gamepad1.right_trigger);

            telemetry.addData("Goal", goalName);
            telemetry.addData("tx", "%.2f", tx);
            telemetry.addData("Distance (in)", "%.1f", distanceIn);
            telemetry.addData("Turret Pos", "%.3f", turretPos);
            telemetry.update();
        }

        limelight.stop();
    }

    // ================= HELPERS =================

    private LLResultTypes.FiducialResult getFiducialById(List<LLResultTypes.FiducialResult> list, int id) {
        if (list == null) return null;
        for (var f : list) if (f != null && f.getFiducialId() == id) return f;
        return null;
    }

    private LLResultTypes.FiducialResult chooseGoal(
            LLResultTypes.FiducialResult blue,
            LLResultTypes.FiducialResult red,
            GoalMode mode) {

        if (mode == GoalMode.FORCE_BLUE) return blue;
        if (mode == GoalMode.FORCE_RED) return red;
        if (blue == null) return red;
        if (red == null) return blue;
        return Math.abs(blue.getTargetXDegrees()) <= Math.abs(red.getTargetXDegrees()) ? blue : red;
    }

    private double estimateDistanceFromAreaOnly(double z) {
        double best = X_MIN_IN, bestCost = Double.POSITIVE_INFINITY;
        for (int i=0;i<300;i++){
            double x = X_MIN_IN+(X_MAX_IN-X_MIN_IN)*i/299.0;
            double c = Math.pow(f_area(x)-z,2);
            if(c<bestCost){bestCost=c;best=x;}
        }
        return best;
    }

    private double f_area(double x) {
        return (1.44509e-9)*Math.pow(x,4)
                -(5.76194e-7)*Math.pow(x,3)
                +(8.48292e-5)*Math.pow(x,2)
                -(5.63797e-3)*x
                +0.152368;
    }

    private double getSizeProxy(Object f) {
        try {
            Method m = f.getClass().getMethod("getTargetArea");
            return ((Number)m.invoke(f)).doubleValue();
        } catch (Exception ignored) {}
        return 0.0;
    }
}
