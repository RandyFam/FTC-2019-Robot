package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoCommands {

    Robot robot = new Robot();

    // class constructor
    public AutoCommands() {

    }

    // Autonomous Methods below



    public void Drive(double power) {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
    }
    public void DriveStop() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    public void TurnLeft(double power){
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
    }

    public void TurnRight(double power){
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
    }
}
