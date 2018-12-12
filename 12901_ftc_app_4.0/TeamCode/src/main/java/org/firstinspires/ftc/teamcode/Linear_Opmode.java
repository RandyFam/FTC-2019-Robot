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

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import for_camera_opmodes.LinearOpModeCamera;

@TeleOp(name = "Basic: Linear OpMode", group = "Linear Opmode")
//@Disabled
public class Linear_Opmode extends LinearOpModeCamera {

    //creating an object from the Testbot's class
    Robot robot = new Robot();

    int ds2 = 2;  // additional downsampling of the image
    // set to 1 to disable further downsampling

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        String colorString = "NONE";

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        robot.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        robot.armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        robot.armMotor2 = hardwareMap.get(DcMotor.class, "arm_motor2");
        //robot.intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        robot.trapdoor = hardwareMap.get(Servo.class, "trapdoor");
        robot.intake1 = hardwareMap.get(Servo.class, "intake1");
        robot.intake2 = hardwareMap.get(Servo.class, "intake2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // Was FORWARD, REVERSE for leftDrive and rightDrive
        robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.armMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.armMotor2.setDirection(DcMotor.Direction.FORWARD);
        robot.intake1.setDirection(Servo.Direction.FORWARD);
        robot.intake2.setDirection(Servo.Direction.FORWARD);
        //robot.intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        double trapDoorPosition = 0;
        double intakePosition = 0;
        if (isCameraAvailable()) {

            setCameraDownsampling(8);
            // parameter determines how downsampled you want your images
            // 8, 4, 2, or 1.
            // higher number is more downsampled, so less resolution but faster
            // 1 is original resolution, which is detailed but slow
            // must be called before super.init sets up the camera

            telemetry.addLine("Wait for camera to finish initializing!");
            telemetry.update();
            startCamera();  // can take a while.
            // best started before waitForStart
            telemetry.addLine("Camera ready!");
            telemetry.update();

            waitForStart();

            // LinearOpMode, so could do stuff like this too.
            /*
            motorLeft.setPower(1);  // drive forward
            motorRight.setPower(1);
            sleep(1000);            // for a second.
            motorLeft.setPower(0);  // stop drive motors.
            motorRight.setPower(0);
            sleep(1000);            // wait a second.
            */

            while (opModeIsActive()) {
                /////////////////////////////////////////////////////////////////////
                // CAMERA CODE
                if (imageReady()) { // only do this if an image has been returned from the camera
                    int redValue = 0;
                    int blueValue = 0;
                    int greenValue = 0;

                    // get image, rotated so (0,0) is in the bottom left of the preview window
                    Bitmap rgbImage;
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

                    for (int x = 0; x < rgbImage.getWidth(); x++) {
                        for (int y = 0; y < rgbImage.getHeight(); y++) {
                            int pixel = rgbImage.getPixel(x, y);
                            redValue += red(pixel);
                            blueValue += blue(pixel);
                            greenValue += green(pixel);
                        }
                    }
                    int color = highestColor(redValue, greenValue, blueValue);

                    switch (color) {
                        case 0:
                            colorString = "RED";
                            break;
                        case 1:
                            colorString = "GREEN";
                            break;
                        case 2:
                            colorString = "BLUE";
                    }

                } else {
                    colorString = "NONE";
                }

                telemetry.addData("Color:", "Color detected is: " + colorString);
                telemetry.update();
                sleep(10);

                // END OF CAMERA CODE
                //////////////////////////////////////////////////////////////////////
                // CONTROLS CODE
                // Setup a variable for each drive wheel to save power level for telemetry
                // Power variables
                double leftPower;
                double rightPower;
                double armPower;
                double armPower2;
                double armSet;

                if(gamepad2.dpad_up){
                    armSet = .5;
                }else if(gamepad2.dpad_down){
                    armSet = -.5;
                }else{
                    armSet = 0;
                }

                // Variables for the game pad
                double driveForward = gamepad1.right_trigger;
                double driveReverse = gamepad1.left_trigger;
                double armValue2 = gamepad2.right_stick_y;
                double armValue = armSet;
                double turn = gamepad1.left_stick_x;

                // Other Variables
                double driveValue = driveForward - driveReverse;
                double requestedIntake = 0;
                boolean buttonDown = false;

                robot.trapdoor.setPosition(trapDoorPosition);

                if (gamepad1.x && trapDoorPosition <= 180) {
                    trapDoorPosition += .25;
                }

                if (gamepad1.y && trapDoorPosition >= 0) {
                    trapDoorPosition -= .25;
                }

                // Intake Code
                robot.intake1.setPosition(intakePosition);
                robot.intake2.setPosition(intakePosition);

                if (gamepad2.a) {
                    intakePosition = .90;
                } else if(gamepad2.b){
                    intakePosition = .10;
                } else {
                    intakePosition = .5;
                }

                // Toggles buttons A and B for intake forward or reverse
            /*if (!gamepad1.a && !gamepad1.b && buttonDown) {
                buttonDown = false;
            }
            else if (gamepad1.a && !buttonDown) {
                buttonDown = true;
                requestedIntake = 1;
            }
            else if (gamepad1.b && !buttonDown) {
                buttonDown = true;
                requestedIntake = -1;
            }*/

                // Makes it so if you press the same button, it turns off the intake
            /*if(requestedIntake == robot.intakeMotor.getPower()){
                robot.intakeMotor.setPower(0);
            }else{
                robot.intakeMotor.setPower(requestedIntake);
            }*/

                // Math for motor power values
                leftPower = Range.clip(driveValue + turn, -1.0, 1.0);
                rightPower = Range.clip(driveValue - turn, -1.0, 1.0);
                armPower = Range.clip(armValue, -0.5, 0.5);
                armPower2 = Range.clip(armValue2, -0.65, 0.65);
                //armPower = armValue;
                //armPower2 = armValue2;

                // Send calculated power to wheel motors
                robot.leftDrive.setPower(leftPower);
                robot.rightDrive.setPower(rightPower);
                robot.armMotor.setPower(armPower);
                robot.armMotor2.setPower(armPower2);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.addData("Trapdoor Position", "Servo1 (%.2f", trapDoorPosition);
                telemetry.addData("Intake Servo Values", "Servo2 & Servo3 (%.2f", intakePosition);
                telemetry.update();
                ///////////////////////////////////////////////////////////////
                // END OF CONTROLS CODE
            }
            stopCamera();

        }
        //////////////////////////////////////////////////////////////////////
        // TELEOP MODE WITHOUT CAMERA
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //robot.trapdoor2.setPosition(1 - trapdoorPosition);

            // Setup a variable for each drive wheel to save power level for telemetry
            // Power variables
            double leftPower;
            double rightPower;
            double armPower;
            double armPower2;
            double armSet;

            if(gamepad2.dpad_up){
                armSet = .5;
            }else if(gamepad2.dpad_down){
                armSet = -.5;
            }else{
                armSet = 0;
            }

            // Variables for the game pad
            double driveForward = gamepad1.right_trigger;
            double driveReverse = gamepad1.left_trigger;
            double armValue2 = gamepad2.right_stick_y;
            double armValue = armSet;
            double turn = gamepad1.left_stick_x;

            // Other Variables
            double driveValue = driveForward - driveReverse;
            double requestedIntake = 0;
            boolean buttonDown = false;

            robot.trapdoor.setPosition(trapDoorPosition);

            if (gamepad1.x && trapDoorPosition <= 180) {
                trapDoorPosition += .25;
            }

            if (gamepad1.y && trapDoorPosition >= 0) {
                trapDoorPosition -= .25;
            }

            // Intake Code
            robot.intake1.setPosition(intakePosition);
            robot.intake2.setPosition(intakePosition);

            if (gamepad2.a) {
                intakePosition = .90;
            } else if(gamepad2.b){
                intakePosition = .10;
            } else {
                intakePosition = .5;
            }

            // Toggles buttons A and B for intake forward or reverse
            /*if (!gamepad1.a && !gamepad1.b && buttonDown) {
                buttonDown = false;
            }
            else if (gamepad1.a && !buttonDown) {
                buttonDown = true;
                requestedIntake = 1;
            }
            else if (gamepad1.b && !buttonDown) {
                buttonDown = true;
                requestedIntake = -1;
            }*/

            // Makes it so if you press the same button, it turns off the intake
            /*if(requestedIntake == robot.intakeMotor.getPower()){
                robot.intakeMotor.setPower(0);
            }else{
                robot.intakeMotor.setPower(requestedIntake);
            }*/

            // Math for motor power values
            leftPower = Range.clip(driveValue + turn, -1.0, 1.0);
            rightPower = Range.clip(driveValue - turn, -1.0, 1.0);
            armPower = Range.clip(armValue, -0.5, 0.5);
            armPower2 = Range.clip(armValue2, -0.65, 0.65);
            //armPower = armValue;
            //armPower2 = armValue2;

            // Send calculated power to wheel motors
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
            robot.armMotor.setPower(armPower);
            robot.armMotor2.setPower(armPower2);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Trapdoor Position", "Servo1 (%.2f", trapDoorPosition);
            telemetry.addData("Intake Servo Values", "Servo2 & Servo3 (%.2f", intakePosition);
            telemetry.update();
        }
    }
}
