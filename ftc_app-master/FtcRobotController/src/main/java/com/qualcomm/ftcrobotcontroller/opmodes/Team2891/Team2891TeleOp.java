/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes.Team2891;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Team2891TeleOp extends OpMode {
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftWinch;
    DcMotor rightWinch;
    DcMotor intakeMotor;
    DcMotor scorerMotor;

    Servo leftTower;
    Servo rightTower;

    int counter = 0;

    double leftTowerPosition;
    double rightTowerPosition;

    float left1;
    float right1;
    boolean intakeIn;
    boolean intakeOut;
    boolean rightArm;
    boolean leftArm;
    float lefty;
    float rightx;
    float right2;
    boolean rightbumper2;
    boolean leftbumper2;
    boolean rightbumper1;
    float righttrigger2;
    float lefttrigger2;
    boolean scorerSlideLeft;
    boolean scorerSlideRight;

    //PID Constants
    double kP=0;
    double kI=0;
    double kD=0;
    double sumError=0;
    double prevError=0;

    public Team2891TeleOp() {

    }
    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftWinch = hardwareMap.dcMotor.get("leftWinch");
        rightWinch = hardwareMap.dcMotor.get("rightWinch");
        leftWinch.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        leftTower = hardwareMap.servo.get("leftTower");
        leftTower.setPosition(0.6118);
        rightTower = hardwareMap.servo.get("rightTower");
        rightTower.setPosition(0.2549);
        scorerMotor = hardwareMap.dcMotor.get("scorerMotor");
    }
    @Override
    public void loop() {
        left1 = -gamepad1.left_stick_y;
        right1 = -gamepad1.right_stick_y;
        lefty = -gamepad1.left_stick_y;
        rightx = -gamepad1.right_stick_x;
        right2 = -gamepad2.right_stick_y;
        rightbumper1 = gamepad1.right_bumper;
        rightbumper2 = gamepad2.right_bumper;
        righttrigger2 = gamepad2.right_trigger;
        leftbumper2 = gamepad2.left_bumper;
        lefttrigger2 = gamepad2.left_trigger;
        intakeIn = gamepad2.a;
        intakeOut = gamepad2.y;
        leftArm = gamepad2.x;
        rightArm = gamepad2.b;
        scorerSlideLeft = gamepad1.x;
        scorerSlideRight = gamepad1.b;

        Methods.doStuff();

        tankDrive(left1, right1, rightbumper1);
        //arcadeDrive(lefty, rightx);
        winchDrive(right2);
        boyzInDaHood(intakeIn, intakeOut);
        ShootArms(leftArm, rightArm);
        ScorerPivot(scorerSlideLeft, scorerSlideRight);
        telemetry.addData("righttrigger", righttrigger2);
        telemetry.addData("lefttrigger", lefttrigger2);
        if (counter >= 2) {
            pivotHangersLeft(leftbumper2, lefttrigger2);
            pivotHangersRight(rightbumper2, righttrigger2);
            counter = 0;
        } else
            counter++;
    }
    @Override
    public void stop() {
    }
    public void tankDrive(double left1, double right1, boolean rightbumper1) //drives right wheels with right joystick and left wheels with left joystick
    {
        if (rightbumper1) {
            leftBack.setPower(-left1);
            leftFront.setPower(-left1);
            rightBack.setPower(-right1);
            rightFront.setPower(-right1);
        } else {
            leftBack.setPower(right1);
            leftFront.setPower(right1);
            rightBack.setPower(left1);
            rightFront.setPower(left1);
        }
    }
    public void arcadeDrive(double throttle, double turn) {
        double left = throttle + turn;
        double right = throttle - turn;
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);
        leftBack.setPower(left);
        leftFront.setPower(left);
        rightBack.setPower(right);
        rightFront.setPower(right);
    }

    public void boyzInDaHood(boolean boyzInDaHoodIn, boolean boyzInDaHoodOut) //drives right wheels with right joystick and left wheels with left joystick
    {
        if (boyzInDaHoodIn) {
            intakeMotor.setPower(1);
        } else if (boyzInDaHoodOut) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
    }
    public void winchDrive(float right2) {
        if (right2 >= 0.15 || right2 <= 0.15) {
            leftWinch.setPower(right2);
            rightWinch.setPower(right2);
        } else {
            leftWinch.setPower(0);
            rightWinch.setPower(0);
        }
    }
    public void pivotHangersRight(boolean rightbumper2, float righttrigger2) {
        if (rightbumper2) {
            rightTowerPosition = (rightTower.getPosition() + 0.01);
        } else if (righttrigger2 > 0.15) {
            rightTowerPosition = (rightTower.getPosition() - 0.005);
        } else {
            rightTowerPosition = rightTower.getPosition();
        }
        if (rightTowerPosition > 1)
            rightTowerPosition = 1;
        if (rightTowerPosition < 0)
            rightTowerPosition = 0;
        rightTower.setPosition(rightTowerPosition);
        telemetry.addData("posRight", rightTower.getPosition());
    }
    public void pivotHangersLeft(boolean leftbumper2, float lefttrigger2) {
        if (leftbumper2) {
            leftTowerPosition = (leftTower.getPosition() - 0.005);
        } else if (lefttrigger2 > 0.15) {
            leftTowerPosition = (leftTower.getPosition() + 0.005);
        } else {
            leftTowerPosition = leftTower.getPosition();
        }
        if (leftTowerPosition > 1)
            leftTowerPosition = 1;
        if (leftTowerPosition < 0)
            leftTowerPosition = 0;
        leftTower.setPosition(leftTowerPosition);
        telemetry.addData("posLeft", leftTower.getPosition());
    }
    public void ShootArms(boolean leftArm, boolean rightArm) {
        if (leftArm) {
            leftTower.setPosition(.2274);
        }
        if (rightArm){
            rightTower.setPosition(.6078);
        }
    }
    public void ScorerPivot (boolean scorerSlideLeft, boolean scorerSlideRight) {
        if (scorerSlideLeft){
            scorerMotor.setPower(-0.5);
        }
        else if (scorerSlideRight){
            scorerMotor.setPower(0.5);
        }
        else {
            scorerMotor.setPower(0);
        }
    }

}
