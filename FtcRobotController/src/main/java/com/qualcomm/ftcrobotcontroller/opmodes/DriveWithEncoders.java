package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public abstract class DriveWithEncoders extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftTank;
    DcMotor rightTank;
    Servo servo;
    //d is distance t is turning degree
    public void go(double d, double t) throws InterruptedException{
        //assign hardware to objects
        leftMotor = hardwareMap.dcMotor.get("left_Motor");
        rightMotor = hardwareMap.dcMotor.get("right_Motor");
        leftTank = hardwareMap.dcMotor.get("left_tank");
        rightTank = hardwareMap.dcMotor.get("right_tank");
        servo = hardwareMap.servo.get("servo");
        //reversing motors as needed
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightTank.setDirection(DcMotor.Direction.FORWARD);
        leftTank.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);


        //defining variables
        double kp;
        double ki;
        double kd;
        double error;
        double previous_error;
        double proportional;
        double intergral;
        double derivitive;
        double ld_speed;
        double rd_speed;
        long dt;
        double PID;
        double targetTicks;
        //setting drive speed
        ld_speed = 0.6;
        rd_speed = 0.6;
        //how often to run the loop
        dt = 20;
        //coefficients for PID loop
        kp = 0.005;
        ki = 0.0025;
        kd = 6;
        //setting variables to zero to use in first loop round
        intergral = 0;
        previous_error = 0;
        //converting distance to rpm
        double r = d / 12.56636;
        //converting rpm to ticks on the encoder
        targetTicks = 1440 * r;
        leftMotor.setPower(ld_speed);
        rightMotor.setPower(rd_speed);
        telemetry.addData("Begining ", "Loop");

        sleep(500);
        while ((leftMotor.getCurrentPosition() < targetTicks) & (rightMotor.getCurrentPosition() < targetTicks)) {
            //error needs to be positive when turning left
            error = (rightMotor.getCurrentPosition() - leftMotor.getCurrentPosition()) - t;
            error /= 1000;
            //proportional which is just error
            proportional = error;
            //intergral art of loop- error over time
            intergral = intergral + error * dt;
            //derivitive which uses slope to correct future error
            derivitive = (error - previous_error) / dt;
            //suming together to create complete correction value
            PID = kp * (proportional + ki * intergral + kd * derivitive);
            //applying corrections to driving
            ld_speed = ld_speed + PID;
            rd_speed = rd_speed - PID;
            ld_speed = Range.clip(ld_speed, 0, 1);
            rd_speed = Range.clip(rd_speed, 0, 1);
            leftMotor.setPower(ld_speed);
            rightMotor.setPower(rd_speed);

            //setting vlues for next loop
            previous_error = error;
            //telemetry stuff
            telemetry.addData("Left Drive", ld_speed);
            telemetry.addData("Right Drive", rd_speed);

            sleep(dt);
        }

    }
    public void halt(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftTank.setPower(0);
        rightTank.setPower(0);
    }
    /*public double gyroCal() throws InterruptedException{
        telemetry.addData("Calibrating Gyro", "Taking First Sample");
        double sample1;
        sample1 = gyro.getRotation();
        telemetry.addData("Calibrating Gyro", "Taking Second Sample");
        double sample2;
        sample2 = gyro.getRotation();
        telemetry.addData("Calibrating Gyro", "Both Samples taken");
        double calStraight = (sample1 + sample2) / 2;
        return calStraight;
    }
    */
}


