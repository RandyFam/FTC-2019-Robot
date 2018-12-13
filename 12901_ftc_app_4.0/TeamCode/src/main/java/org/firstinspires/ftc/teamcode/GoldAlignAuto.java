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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "GoldAlign", group = "DogeCV")

public class GoldAlignAuto extends OpMode {

    Robot robot = new Robot(); // uses the Robot's hardware
    AutoCommands auto = new AutoCommands();

    // Detector object
    private GoldAlignDetector detector;


    @Override
    public void init() {

        //robot.init(hardwareMap);

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











        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!


    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //robot.leftDrive.setPower(1);
        if (!detector.getAligned()) {
           // auto.TurnRight(.8); // turn until detector is aligned
        }
/*        while (detector.getAligned()) {
            //robot.leftDrive.setPower(1);
            //auto.Drive(.8); // drive forward
            sleep(1000);
            //auto.Drive(-.6); // drive back
            sleep(1000);
            //auto.TurnLeft(.8); // turn left
            sleep(1000);
            //auto.Drive(.8); // drive to depot
            sleep(2000);
           // auto.DriveStop();
        }*/
    }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {
        telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", robot.leftDrive.getPower(), robot.rightDrive.getPower());
        robot.leftDrive.setPower(1);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Disable the detector
        detector.disable();
    }

}
