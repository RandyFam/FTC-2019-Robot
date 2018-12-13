package org.firstinspires.ftc.teamcode;

public class AutoCommands {

    Robot robot = new Robot();

    // class constructor
    public AutoCommands() {

    }

    // Autonomous Methods below

    public void drive(double power) {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
    }
    public void driveStop() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
}
