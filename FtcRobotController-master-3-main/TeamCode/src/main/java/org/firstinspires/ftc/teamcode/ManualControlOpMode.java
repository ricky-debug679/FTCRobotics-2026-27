package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

@TeleOp(name = "Turret: Manual + Corrected Distance Shooter", group = "TeleOp")
public class ManualControlOpMode extends LinearOpMode {

    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    private static final double GATE_CLOSED = 0.15;
    private static final double GATE_OPEN   = 0.65;
    private static final double TURRET_MANUAL_PWR = 0.35;
    private static final double TURRET_PID_KP = 0.02; // tune for smooth auto-lock

    // Camera & target heights for horizontal distance correction
    private static final double CAMERA_HEIGHT_IN = 10.0; // inches
    private static final double TAG_HEIGHT_IN    = 30.0; // inches
    private static final double CAMERA_TILT_DEG  = 25.0; // tilt up

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor intakeMotor;
    private CRServo transferServo;
    private Servo gateServo;
    private DcMotor shooter1, shooter2;
    private DcMotorEx turret;
    private Limelight3A limelight;
    private VoltageSensor voltageSensor;

    private int currentGoalID = BLUE_GOAL_ID;
    private boolean dpadUpPrev = false;

    @Override
    public void runOpMode() {

        // ===== Hardware Map =====
        leftFront  = hardwareMap.get(DcMotor.class, "left front");
        leftBack   = hardwareMap.get(DcMotor.class, "left back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        rightBack  = hardwareMap.get(DcMotor.class, "right back");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        transferServo = hardwareMap.get(CRServo.class, "transferServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(GATE_OPEN);

        shooter1 = hardwareMap.get(DcMotor.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "Shooter2");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        waitForStart();

        while (opModeIsActive()) {

            // ===== Drive =====
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double max = Math.max(1.0,
                    Math.max(Math.abs(drive + strafe + turn),
                            Math.max(Math.abs(drive - strafe + turn),
                                    Math.max(Math.abs(drive - strafe - turn),
                                            Math.abs(drive + strafe - turn)))));

            leftFront.setPower((drive + strafe + turn) / max);
            leftBack.setPower((drive - strafe + turn) / max);
            rightFront.setPower((drive - strafe - turn) / max);
            rightBack.setPower((drive + strafe - turn) / max);

            // ===== Goal Toggle =====
            boolean dpadUpPressed = gamepad1.dpad_up;
            if (dpadUpPressed && !dpadUpPrev) {
                currentGoalID = (currentGoalID == BLUE_GOAL_ID) ? RED_GOAL_ID : BLUE_GOAL_ID;
            }
            dpadUpPrev = dpadUpPressed;

            // ===== Limelight: Corrected Distance =====
            boolean seesGoal = false;
            double distanceInches = -1;
            double bearingDeg = 0;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                for (LLResultTypes.FiducialResult f : tags) {
                    if (f.getFiducialId() == currentGoalID) {
                        double tzInches = f.getTz() * 39.37; // meters -> inches
                        double camPitchRad = Math.toRadians(CAMERA_TILT_DEG);
                        // horizontal distance correction
                        distanceInches = tzInches * Math.cos(camPitchRad) - (TAG_HEIGHT_IN - CAMERA_HEIGHT_IN);
                        bearingDeg = f.getTx(); // horizontal angle to tag
                        seesGoal = true;
                        break;
                    }
                }
            }

            // ===== Turret Auto-Lock =====
            double turretPower = 0.0;
            if (seesGoal) {
                turretPower = bearingDeg * TURRET_PID_KP;
            }
            if (gamepad1.dpad_left)  turretPower -= TURRET_MANUAL_PWR;
            if (gamepad1.dpad_right) turretPower += TURRET_MANUAL_PWR;
            turret.setPower(Range.clip(turretPower, -1.0, 1.0));

            // ===== Shooter =====
            double shooterPower = 0.0;
            if (seesGoal && distanceInches > 0) {
                shooterPower = integratedShooterFormula(distanceInches);
            }
            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            // ===== Intake + Transfer =====
            if (gamepad1.circle) {
                intakeMotor.setPower(1.0);
                transferServo.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.4);
                transferServo.setPower(1.0);
            }

            gateServo.setPosition(gamepad1.right_bumper ? GATE_OPEN : GATE_CLOSED);

            // ===== Telemetry =====
            telemetry.addData("Current Goal ID", currentGoalID);
            telemetry.addData("Distance (inches)", distanceInches);
            telemetry.addData("Bearing (deg)", bearingDeg);
            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.addData("Battery", "%.1f V", voltageSensor.getVoltage());
            telemetry.update();
        }

        limelight.stop();
    }

    // ===== Shooter Formula =====
    private double integratedShooterFormula(double x) {
        double power =
                0.95 * (
                        0.00000122124 * Math.pow(x, 4)
                                - 0.000320651 * Math.pow(x, 3)
                                + 0.0309334 * Math.pow(x, 2)
                                - 1.29708 * x
                                + 20.51573
                );
        return Range.clip(power, 0.0, 1.0);
    }
}
