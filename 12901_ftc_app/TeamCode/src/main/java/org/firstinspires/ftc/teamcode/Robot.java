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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    /* Public OpMode members. */
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armMotor;
    public DcMotor armMotor2;
    //public DcMotor intakeMotor;

    public Servo trapdoor;
    public Servo trapdoor2;
    public Servo intake1;
    public Servo intake2;
    /* local OpMode members. */
    public HardwareMap hwMap;

    /* Constructor */
    public Robot() {
        // this is where all of the code that makes this class.
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
        armMotor2.setPower(0);
        //intakeMotor.setPower(0);
        //trapdoor.setPosition(180);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
    ------------------------------------------------------------
    ------------------------------------------------------------
    ------------------------------------------------------------
    ------------------AUTO COMMANDS HERE------------------------
    ------------------------------------------------------------
    ------------------------------------------------------------
    ------------------------------------------------------------
     */

    public void Drive(double power) {
        leftDrive.setPower(-power);
        rightDrive.setPower(-power);
    }

    public void Stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void TurnRight(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
    }

    public void TurnLeft(double power) {
        leftDrive.setPower(-power);
        rightDrive.setPower(power);
    }

    public void AutoArm(double power) { //POSITIVE ALWAYS MOVES OUT
        //Negative power moves arm out
        //Positive power moves arm inwards
        //Max .65
        armMotor2.setPower(-power * .65);
    }

    public void AutoWrist(double power) { //POSITIVE ALWAYS MOVES OUT
        //Positive power moves arm out
        //Negative power moves arm in
        //Max .5
        armMotor.setPower(power * .5);
    }

    public void ArmOut(double power) { //POSITIVE ALWAYS MOVES OUT
        armMotor2.setPower(-power * .65);
        armMotor.setPower(power * .5);
    }

    public void Drop() {
        trapdoor.setPosition(0);
        trapdoor2.setPosition(1);
    }

    public void Close() {
        trapdoor.setPosition(.5);
        trapdoor2.setPosition(.5);
    }

    public void Outtake() {
        intake1.setPosition(.9);
        intake2.setPosition(.9);
    }

    public void Intake() {
        intake1.setPosition(.1);
        intake2.setPosition(.1);
    }
}

