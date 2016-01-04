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

    //defining drivetrain motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    //defining winch motors
    DcMotor leftWinch;
    DcMotor rightWinch;

    DcMotor intakeMotor;

    //defining towers(the hanging servos)
    Servo leftTower;
    Servo rightTower;
    Servo intakeServo;


    //misc variables
    double winchPower;
    int counter = 0;
    double leftTowerPosition;
    double rightTowerPosition;

    //joystick variables
    float left1;
    float right1;
    boolean intakeIn;
    boolean intakeOut;
    boolean intakeLeft;
    boolean intakeRight;

    // arcade drive
    float lefty;
    float rightx;

    float left2; //intake servo tilt
    float right2; // winch motor

    boolean rightbumper2; // winch down
    boolean leftbumper2; // winch down
    float righttrigger2; // winch up
    float lefttrigger2; // winch up


    public Team2891TeleOp() {

    }

    @Override
    public void init() {
        //matching motors to configured motors on phone


        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftWinch = hardwareMap.dcMotor.get("leftWinch");
        rightWinch = hardwareMap.dcMotor.get("rightWinch");
        leftWinch.setDirection(DcMotor.Direction.REVERSE);
        //matching servos to configured servos

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        leftTower = hardwareMap.servo.get("leftTower");
        leftTower.setPosition(0.6745);
        rightTower = hardwareMap.servo.get("rightTower");
        rightTower.setPosition(0.2078);
        intakeServo = hardwareMap.servo.get("intakeServo");
        intakeServo.setPosition(1-0.775);

    }

    @Override
    public void loop() {
        left1 = -gamepad1.left_stick_y;
        right1 = -gamepad1.right_stick_y;

        //arcade
        lefty = -gamepad1.left_stick_y;
        rightx = -gamepad1.right_stick_x;

        left2 = -gamepad2.left_stick_x;
        right2 = -gamepad2.right_stick_y;

        rightbumper2 = gamepad2.right_bumper;
        righttrigger2 = gamepad2.right_trigger;
        leftbumper2 = gamepad2.left_bumper;
        lefttrigger2 = gamepad2.left_trigger;
        intakeIn = gamepad2.a;
        intakeOut = gamepad2.y;
        intakeLeft = gamepad2.x;
        intakeRight = gamepad2.b;

        //tankDrive(left1, right1);
        arcadeDrive(lefty, rightx);
        winchDrive(right2);
        Intake(intakeIn, intakeOut);
        intake(intakeLeft, intakeRight);

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

    public void tankDrive(double left1, double right1) //drives right wheels with right joystick and left wheels with left joystick
    {
        leftBack.setPower(right1);
        leftFront.setPower(right1);
        rightBack.setPower(left1);
        rightFront.setPower(left1);
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

    public void Intake(boolean intakeIn, boolean intakeOut) //drives right wheels with right joystick and left wheels with left joystick
    {
        if (intakeIn) {
            intakeMotor.setPower(1);
        } else if (intakeOut) {

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

        //truncate large or small values
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

    public void intake(boolean intakeLeft, boolean intakeRight) {
        telemetry.addData("ServoIntakePosition", intakeServo.getPosition());
        if (intakeLeft){
            intakeServo.setPosition(1-0.775-.12);
        }
        else if (intakeRight){
            intakeServo.setPosition(1-0.775+.12);
        }
        else {
            intakeServo.setPosition(1-0.775);
        }

    }
}
