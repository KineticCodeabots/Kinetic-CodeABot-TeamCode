package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class Turret_Test extends OpMode{

    private DcMotorEx FlywheelLeft;
    private DcMotorEx FlywheelRight;

    public void handle_gamepad(){
        if (gamepad1.a) {
            FlywheelLeft.setVelocity(1000);
            FlywheelRight.setVelocity(1000);
        } else if (gamepad1.b) {
            FlywheelLeft.setVelocity(3000);
            FlywheelRight.setVelocity(3000);
        } else if (gamepad1.y) {
            FlywheelLeft.setVelocity(500);
            FlywheelLeft.setVelocity(500);
        }
    }

    @Override
    public void init() {
        FlywheelLeft = hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
        FlywheelRight = hardwareMap.get(DcMotorEx.class, "FlywheelRight");
        FlywheelLeft.setDirection(DcMotorEx.Direction.FORWARD);
        FlywheelRight.setDirection(DcMotorEx.Direction.REVERSE);
        FlywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Joel", "Welcome to the jungle!");
    }

    @Override
    public void loop() {
        handle_gamepad();
    }


}

