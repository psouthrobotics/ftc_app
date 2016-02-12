package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

public class Tank extends OpMode {
    //drive motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor rightTank;
    DcMotor leftTank;
    Servo servo;
    Servo flipper;
    UltrasonicSensor front;
    //drive values
    double left;
    double right;
    double a = 1;
    //scale for expo




    @Override
    public void init() {

        //Mapping physical motors to variable
        leftMotor = hardwareMap.dcMotor.get("left_Motor");
        rightMotor = hardwareMap.dcMotor.get("right_Motor");
        leftTank = hardwareMap.dcMotor.get("left_tank");
        rightTank = hardwareMap.dcMotor.get("right_tank");
        servo = hardwareMap.servo.get("servo");
        flipper = hardwareMap.servo.get("flip");
        front = hardwareMap.ultrasonicSensor.get("front");

        leftMotor.setDirection(DcMotor.Direction.REVERSE );
        rightTank.setDirection(DcMotor.Direction.REVERSE);
        leftTank.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {
        //getting drive values from controller
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        //clipping values so they dont exceed 100% on motors
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);
        //scale drive values for easier controlling
        expoL();
        expoR();

        if (gamepad2.x)
            servo.setPosition(1);
        if (gamepad2.b)
            servo.setPosition(0);
        if (gamepad2.y)
            servo.setPosition(0.5);
        if (gamepad2.dpad_down)
            a = 0.3;
        if (gamepad2.dpad_up)
            a = 1;
        if (gamepad2.right_bumper)
            flipper.setPosition(0.8);
        if (gamepad2.left_bumper)
            flipper.setPosition(0.2);
        //set drive values for
        left = left * a;
        right = right * a;
        leftMotor.setPower(left);
        rightMotor.setPower(right);
        leftTank.setPower(left);
        rightTank.setPower(right);



        //telemetry
        telemetry.addData("Left Motor power", left);
		telemetry.addData("Right Motor power", right);
        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.addData("Distance in Front", front.getUltrasonicLevel());
        telemetry.addData("Speed", a);
    }
    //scale left drive value
    public void expoL() {
        if (left > 0)
            left = left*left;
        if (left <= 0)
            left = -(left*left);
    }
    //scale right drive value
    public void expoR() {
        if (right > 0)
            right = right*right;
        if (right <= 0)
            right = -(right*right);
    }

}