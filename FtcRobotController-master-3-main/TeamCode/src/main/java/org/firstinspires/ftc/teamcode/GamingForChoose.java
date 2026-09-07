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

import java.util.List;

@TeleOp(name = "Gaming For Choose - Fixed", group = "TeleOp")
public class GamingForChoose extends LinearOpMode {

    // Constants
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    // Servo Positions (Must be between 0.0 and 1.0)
    private static final double GATE_CLOSED = 0.3;
    private static final double GATE_OPEN   = .9; // Adjusted from negative value

    private static final double TURRET_MANUAL_PWR = 0.35;

    // Distance Solver Constants
    private static final double X_MIN_IN = 20.0;
    private static final double X_MAX_IN = 170.0;

    // Hardware
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor intakeMotor;
    private CRServo transferServo;
    private Servo gateServo;
    private DcMotor shooter1, shooter2;
    private DcMotorEx turret;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // --- Drivetrain ---
        leftFront  = hardwareMap.get(DcMotor.class, "left front");
        leftBack   = hardwareMap.get(DcMotor.class, "left back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        rightBack  = hardwareMap.get(DcMotor.class, "right back");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Intake & Transfer ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intake motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferServo = hardwareMap.get(CRServo.class, "transferServo");

        // --- Gate ---
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);

        gateServo.setPosition(GATE_CLOSED);


        // --- Shooters ---
        shooter1 = hardwareMap.get(DcMotor.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "Shooter2");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Turret ---
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        telemetry.addLine("Ready. Servos Initialized.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // 1. DRIVE (Mecanum)
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double lf = drive + strafe + turn;
            double lb = drive - strafe + turn;
            double rf = drive - strafe - turn;
            double rb = drive + strafe - turn;

            double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));
            leftFront.setPower(lf / max);
            leftBack.setPower(lb / max);
            rightFront.setPower(rf / max);
            rightBack.setPower(rb / max);

            // 2. INTAKE & TRANSFER
            intakeMotor.setPower(gamepad1.left_trigger);

            // Logic: Default 1.0, Reverse if Cross is held
            double transferPwr = 1.0;
            if (gamepad1.cross) {
                transferPwr = -1.0;
            }
            transferServo.setPower(transferPwr);

            // 3. GATE (Triangle Button)
            if (gamepad1.triangle) {
                gateServo.setPosition(GATE_OPEN); // Moves to push outwards
            }if(gamepad1.triangleWasReleased()) {
                gateServo.setPosition(GATE_CLOSED); // Returns to default

            }

            // 4. MANUAL TURRET (D-Pad)
            double turretPower = 0.0;
            if (gamepad1.dpad_left)  turretPower = -TURRET_MANUAL_PWR;
            else if (gamepad1.dpad_right) turretPower = TURRET_MANUAL_PWR;
            turret.setPower(turretPower);

            // 5. MANUAL SHOOTER (Right Trigger)
            double shootPwr = gamepad1.right_trigger;
            shooter1.setPower(shootPwr);
            shooter2.setPower(shootPwr);

            // 6. LIMELIGHT DISTANCE DATA
            double distanceIn = -1;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                for (LLResultTypes.FiducialResult f : tags) {
                    if (f.getFiducialId() == BLUE_GOAL_ID || f.getFiducialId() == RED_GOAL_ID) {
                        distanceIn = estimateDistanceFromAreaOnly(f.getTargetArea());
                        break;
                    }
                }
            }

            // 7. TELEMETRY
            telemetry.addData("Gate Pos", gateServo.getPosition());
            telemetry.addData("Transfer Pwr", transferPwr);
            telemetry.addData("Shooter Pwr", shootPwr);
            telemetry.addData("Distance (In)", (distanceIn == -1) ? "NO TAG" : String.format("%.2f", distanceIn));
            telemetry.addData("ServoPosition", gateServo.getPosition());
            telemetry.update();
        }
        limelight.stop();
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
        return (1.44509e-9) * Math.pow(x, 4) - (5.76194e-7) * Math.pow(x, 3) + (8.48292e-5) * Math.pow(x, 2) - (5.63797e-3) * x + 0.152368;
    }
}