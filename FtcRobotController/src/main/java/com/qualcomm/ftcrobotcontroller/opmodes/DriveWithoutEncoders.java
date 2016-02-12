package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;


public class DriveWithoutEncoders extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftTank;
    DcMotor rightTank;

    GyroSensor gyro;

    public DriveWithoutEncoders() {

    }
    //To satisfy interface
    public void runOpMode(){}

    //t is time x is direction
    public void goT(double time, double turningRate) throws InterruptedException{

        mapHardware();
        //reversing motors as needed
        setDirection();
        //setting straight value
        //below is left above is right
        double straight = 575 + turningRate;
        //setting drive speed
        double lSpeed = 0.6;
        double rSpeed = lSpeed;
        //How long to sleep between loop  cycles to help with math
        long dt = 20;
        //coefficients for PID loop
        //                      P        I    D
        double[] pidValues = {0.0025, 0.0017, 3};
        //start time to compare against so loop only runs for so long
        double start_time = System.currentTimeMillis();
        //setting variables to zero to use in first loop round to avoid NULL errors
        double integral = 0;
        double previous_error = 0;
        //timer to run loop for given amount of time
        while (System.currentTimeMillis() - start_time < time) {
            //error is how far off a straight value is to use for calculating corrections
            double error = straight - gyro.getRotation();
            //dividing error because motor speed is in percentage
            error = error / 1000;
            //proportional factor so correction is relative size of error
            double proportional = error;
            //integral helps deal with drift by calculating error over time and builds as the loop goes to deal with uncorrected error
            integral = integral + error * dt;
            //derivative which uses slope of the error to correct future error and to prevent overshooting
            double derivative = (error - previous_error) / dt;
            //summing together to create complete correction value and multiplying by coefficients
            double PID = pidValues[0] * proportional + pidValues[1] * integral + pidValues[2] * derivative;
            //applying corrections to driving so we go straight
            lSpeed = lSpeed + PID;
            rSpeed = rSpeed - PID;
            lSpeed = Range.clip(lSpeed, 0, 1);
            rSpeed = Range.clip(rSpeed, 0, 1);
            leftMotor.setPower(lSpeed);
            rightMotor.setPower(rSpeed);
            leftTank.setPower(lSpeed);
            rightTank.setPower(rSpeed);

            //setting values for next loop so integral roles over
            previous_error = error;
            //telemetry stuff
            telemetry.addData("Left Drive", lSpeed);
            telemetry.addData("Right Drive", rSpeed);
            telemetry.addData("Rotation", gyro.getRotation());

            sleep(dt);
        }

    }
    public void halt(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftTank.setPower(0);
        rightTank.setPower(0);
    }
    public void mapHardware(){
        leftMotor = hardwareMap.dcMotor.get("left_Motor");
        rightMotor = hardwareMap.dcMotor.get("right_Motor");
        leftTank = hardwareMap.dcMotor.get("left_tank");
        rightTank = hardwareMap.dcMotor.get("right_tank");
        gyro = hardwareMap.gyroSensor.get("gy");
    }
    public void setDirection(){
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightTank.setDirection(DcMotor.Direction.FORWARD);
        leftTank.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
    }

}


