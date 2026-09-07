package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "BASIC: Drive(RS) + Intake + CRServo (PS4)", group = "TeleOp")
public class BasicDriveIntakeServo extends LinearOpMode {

    // ===== Hardware =====
    private DcMotor leftFront, leftBack, rightFront, rightBack, Shooter1, Shooter2;
    private DcMotor intake;
    private CRServo servo;   // CONTINUOUS ROTATION SERVO

    @Override
    public void runOpMode() {

        // Drivetrain
        leftFront  = hardwareMap.get(DcMotor.class, "left front");
        leftBack   = hardwareMap.get(DcMotor.class, "left back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        rightBack  = hardwareMap.get(DcMotor.class, "right back");

        // Intake + CRServo
        intake = hardwareMap.get(DcMotor.class, "intake");
        servo  = hardwareMap.get(CRServo.class, "servo");
        Shooter1 = hardwareMap.get(DcMotor.class, "Shooter1");
        Shooter2 = hardwareMap.get(DcMotor.class, "Shooter2");

        // Motor directions (adjust if needed)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("READY");
        telemetry.addLine("RS Y = drive | RS X = turn");
        telemetry.addLine("R2 = intake");
        telemetry.addLine("Hold X = servo forward | Hold △ = servo reverse");
        telemetry.addLine("DpadLeft = swerve left");
        telemetry.addLine("DpadRight = swerve right");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // DRIVETRAIN (RIGHT STICK)
            // =========================
            double drive = -gamepad1.right_stick_y;
            double turn  =  gamepad1.right_stick_x;



            double leftPower  = Range.clip(drive + turn, -1, 1);
            double rightPower = Range.clip(drive - turn, -1, 1);


            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);

            if(gamepad1.dpad_left){
                leftFront.setPower(leftPower);
                rightBack.setPower(leftPower);
                rightFront.setPower(rightPower);
                rightBack.setPower(rightPower);
            }
            if(gamepad1.dpad_right){
                leftFront.setPower(rightPower);
                rightBack.setPower(rightPower);
                rightFront.setPower(leftPower);
                rightBack.setPower(leftPower);
            }

            // =========================
            // INTAKE (R2)
            // =========================
            double intakePower = Range.clip(gamepad1.right_trigger, 0, 1);
            intake.setPower(intakePower);

            if(gamepad1.square){
                intakePower = -1.0;
            }

            // =========================
            // CR SERVO CONTROL
            // =========================
            double servoPower = 0.0;

            if (gamepad1.cross) {
                servoPower = 1.0;     // spin forward

            } else if (gamepad1.triangle) {
                servoPower = -1.0;    // spin reverse
            }

            servo.setPower(servoPower);

            // =========================
            // TELEMETRY
            // =========================
            telemetry.addData("Drive", drive);
            telemetry.addData("Turn", turn);
            telemetry.addData("Intake", "%.2f", intakePower);
            telemetry.addData("Servo Power", "%.2f", servoPower);
            telemetry.update();
        }

        // Stop everything
        intake.setPower(0);
        servo.setPower(0);
    }
}
