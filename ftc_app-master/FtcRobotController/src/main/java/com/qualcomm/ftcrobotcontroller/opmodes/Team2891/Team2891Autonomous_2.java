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
    Servo dongerLord;

    double ticksPerRotation = 1120;
    double pi = 3.1415926535897932384626;
    double diameter = 4;
    double circumference = pi*diameter;
    double gearRatio = 54/30;
    double targetDistance = 50*48/35;
    double P;
    double I;
    double D;
    double kP = 0.2;
    double kI = 0;
    double kD = 0;
    double error = 0;
    double sumError = 0;
    double changeError = 0;
    double prevError = 0;
    double PID;
    double PIDOutput;

    public Team2891Autonomous_2() {
    }
    @Override
    public void init() {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        leftTower = hardwareMap.servo.get("leftTower");
        leftTower.setPosition(0.6745);
        rightTower = hardwareMap.servo.get("rightTower");
        rightTower.setPosition(0.2078);

        dongerLord = hardwareMap.servo.get("dongerLord");

    }

    @Override
    public void loop() {
        telemetry.addData("Encoder Value", ticksPerRotation * (1 / circumference) * gearRatio * targetDistance);
        telemetry.addData("Encoder Current Value", rightFront.getCurrentPosition());

        PIDOutput = calculatePID(rightFront.getCurrentPosition(), ticksPerRotation * (1 / circumference) * gearRatio * targetDistance);

        if (Math.abs(leftFront.getCurrentPosition()) < ticksPerRotation * (1 / circumference) * gearRatio * targetDistance) {
            leftFront.setPower(PIDOutput);
            leftBack.setPower(PIDOutput);
            rightFront.setPower(PIDOutput);
            rightBack.setPower(PIDOutput);
        } else if (Math.abs(leftFront.getCurrentPosition()) > ticksPerRotation * (1 / circumference) * gearRatio * targetDistance){
            leftFront.setPower(-PIDOutput);
            leftBack.setPower(-PIDOutput);
            rightFront.setPower(-PIDOutput);
            rightBack.setPower(-PIDOutput);
        }
        else{
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }

    public double calculatePID(double current, double setpoint) {
            error = setpoint - current;
            sumError = sumError + error;
            changeError = (prevError - error);
            P = kP * error;
            I = sumError * kI;
            D = kD * changeError;
            PID = P + I + D;
            prevError = error;

            telemetry.addData("PID", PID);
            if (PID>1){
                return 1;
            }
            if (PID<1){
                return -1;
            }

            return PID;
    }
    public void stop() {

    }

}
