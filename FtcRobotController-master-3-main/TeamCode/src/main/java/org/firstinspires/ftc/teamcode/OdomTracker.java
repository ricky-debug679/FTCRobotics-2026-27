package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Odom_Tracker_Sample")
public class OdomTracker extends LinearOpMode {

    // 1. Define Hardware
    private DcMotor verticalEncoder, horizontalEncoder;
    private IMU imu;

    // 2. Constants (Adjust these for your robot!)
    // Formula: Ticks per rev / (Wheel Diameter * PI)
    final double TICKS_PER_INCH = 326.1;

    // 3. Position Variables
    double globalX = 0;
    double globalY = 0;
    int prevVerticalTicks = 0;
    int prevHorizontalTicks = 0;

    @Override
    public void runOpMode() {
        // Initialize Motors as Encoders
        verticalEncoder = hardwareMap.get(DcMotor.class, "left_back"); // Example port
        horizontalEncoder = hardwareMap.get(DcMotor.class, "right_back"); // Example port

        // Reset Encoders to 0
        verticalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize goBILDA IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized. Push the robot!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // A. Get Current Values
            int currentVertical = verticalEncoder.getCurrentPosition();
            int currentHorizontal = horizontalEncoder.getCurrentPosition();
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // B. Calculate Deltas (Change since last loop)
            double deltaV = (currentVertical - prevVerticalTicks) / TICKS_PER_INCH;
            double deltaH = (currentHorizontal - prevHorizontalTicks) / TICKS_PER_INCH;

            // C. The Rotation Matrix (Local to Global)
            // This translates robot-relative movement into field-relative movement
            double deltaX = deltaH * Math.cos(heading) - deltaV * Math.sin(heading);
            double deltaY = deltaH * Math.sin(heading) + deltaV * Math.cos(heading);

            // D. Update Global Position
            globalX += deltaX;
            globalY += deltaY;

            // E. Save values for next loop
            prevVerticalTicks = currentVertical;
            prevHorizontalTicks = currentHorizontal;

            // Telemetry Output
            telemetry.addData("X (Inches)", "%.2f", globalX);
            telemetry.addData("Y (Inches)", "%.2f", globalY);
            telemetry.addData("Heading (Deg)", "%.2f", Math.toDegrees(heading));
            telemetry.update();
        }
    }
}