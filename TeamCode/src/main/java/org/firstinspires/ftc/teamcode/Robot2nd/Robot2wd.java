package org.firstinspires.ftc.teamcode.Robot2nd;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Robot2wd extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx LeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("LeftMotor");
        DcMotorEx RightMotor = (DcMotorEx) hardwareMap.dcMotor.get("RightMotor");
        double MaxSpeed = 1;

        RightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Y stick value is reversed
            double x = gamepad1.right_stick_x;

            // Joel's deadzone code

            if (Math.abs(y) < 0.1) y = 0;
            if (Math.abs(x) < 0.1) x = 0;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double LeftPower = (y + x) / denominator * MaxSpeed;
            double RightPower = (y - x) / denominator * MaxSpeed;

            LeftMotor.setPower(LeftPower);
            RightMotor.setPower(RightPower);

            telemetry.addData("Stick X", x);
            telemetry.addData("Stick Y", -y);
            telemetry.addData("Left Power", LeftPower);
            telemetry.addData("Right Power", RightPower);
            telemetry.addData("Robot Speed cm/s", LeftMotor.getVelocity() * 282.74/60);
            telemetry.update();
        }
    }
}