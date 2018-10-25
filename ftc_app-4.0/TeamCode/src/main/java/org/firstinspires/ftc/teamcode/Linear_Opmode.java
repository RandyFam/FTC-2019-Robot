/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Linear OpMode", group = "Linear Opmode")
//@Disabled
public class Linear_Opmode extends LinearOpMode {

    // Declare OpMode members.

    //creating an object from the Testbot's class
    Testbot robot = new Testbot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        robot.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        robot.armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        robot.intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // Was FORWARD, REVERSE for leftDrive and rightDrive
        robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.armMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            // Power variables
            double leftPower;
            double rightPower;
            double armPower;

            // Variables for the game pad
            double driveForward = gamepad1.right_trigger;
            double driveReverse = gamepad1.left_trigger;
            double armValue = gamepad1.right_stick_y;
            double turn = gamepad1.left_stick_x;
            boolean intakeOn = gamepad1.x;
            boolean intakeOff = gamepad1.a;
            boolean intakeReverse = gamepad1.b;

            // Other Variables
            boolean firstRun = true;
            double driveValue = driveForward - driveReverse;

            // Intake on when X button is pressed
            if (intakeOn) {
                robot.intakeMotor.setPower(.8);
            }
            // Intake off when A button is pressed
            if (intakeOff) {
                robot.intakeMotor.setPower(0);
            }
            // Intake reverse when B button is pressed
            if (intakeReverse) {
                robot.intakeMotor.setPower(-.8);
            }

            leftPower = Range.clip(driveValue + turn, -1.0, 1.0);
            rightPower = Range.clip(driveValue - turn, -1.0, 1.0);
            armPower = Range.clip(armValue, -1.0, 1.0);

            // Send calculated power to wheel motors
            robot.leftDrive.setPower(-leftPower);
            robot.rightDrive.setPower(-rightPower);
            robot.armMotor.setPower(armPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
