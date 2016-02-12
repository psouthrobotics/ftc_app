package com.qualcomm.ftcrobotcontroller.opmodes;

public class Autonomous {
    Drive drive = new Drive();
    public void runOpMode() throws InterruptedException{

        drive.go(2000);
        drive.haltT(2000);
        drive.goT(2000, 300);
        drive.haltT(2000);
        drive.goTB(2000, 300);
        drive.goB(2000);
        drive.haltT(2000);
        drive.driveE(24);
    }
}
