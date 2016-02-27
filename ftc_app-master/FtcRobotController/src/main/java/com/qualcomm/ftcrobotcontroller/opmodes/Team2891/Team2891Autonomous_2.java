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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;



public class Team2891Autonomous_2 extends OpMode {

    /**
     * Constructor
     */

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intakeMotor;

    Servo leftTower;
    Servo rightTower;


    double ticksPerRotation = 1120;
    double pi = 3.1415926535897932384626;
    double diameter = 4;
    double circumference = pi*diameter;
    double gearRatio = 54/30;
    double distanceTraveled = 110;


    public Team2891Autonomous_2() {

    }

    @Override
    public void init() {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        leftTower = hardwareMap.servo.get("leftTower");
        leftTower.setPosition(0.6745);
        rightTower = hardwareMap.servo.get("rightTower");
        rightTower.setPosition(0.2078);
    }

    @Override
    public void loop() {
        if (rightFront.getCurrentPosition() > ticksPerRotation*(1/circumference)*gearRatio*distanceTraveled*-1){
            leftFront.setPower(0.25);
            leftBack.setPower(0.25);
            rightFront.setPower(-0.25);
            rightBack.setPower(-0.25);
            intakeMotor.setPower(-1);
        }
        else{
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            intakeMotor.setPower(0);
        }
        /*telemetry.addData("Ticks RightFront",rightFront.getCurrentPosition());
        telemetry.addData("Ticks RightBack",rightBack.getCurrentPosition());
        telemetry.addData("Ticks LeftFront",leftFront.getCurrentPosition());
        telemetry.addData("Ticks LeftBack",leftBack.getCurrentPosition());*/
        telemetry.addData("motorValue:", ticksPerRotation*(1/circumference)*gearRatio*distanceTraveled*-1);
    }

    @Override
    public void stop() {

    }

}
