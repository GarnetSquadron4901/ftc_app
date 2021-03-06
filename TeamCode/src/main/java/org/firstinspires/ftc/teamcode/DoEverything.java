/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.SynchronousQueue;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "DoEverything")  // @Autonomous(...) is the other common choice
//@Disabled
public class DoEverything extends LinearOpMode
{
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    DcMotor shooter;
    double m1 = 0;
    double m2 = 0;
    CompassSensor compass;
    ModernRoboticsI2cGyro gyro;
    DeviceInterfaceModule dInterface;

    final static double DIAMETER = 4;
    final static int ENCODER = 280;
    final static double CIRCUMFERENCE = Math.PI * DIAMETER;

    private ElapsedTime runtime = new ElapsedTime();


    long currentTime, diff, startTime;


    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Status", "Initialized");


        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        shooter = hardwareMap.dcMotor.get("shooter");


        dInterface = hardwareMap.deviceInterfaceModule.get("deviceInterfaceModule");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();



        //spin up shooter
        //shoot
        //go forwards until white line is found
        //turn 90 degrees
        //go forwards until close to wall
        //determine which button to press
        //press correct button
        //turn -90 degrees
        //go forwards until white line is found
        //turn 90 degrees
        //go forwards until close to wall
        //determine which button to press
        //press correct button
        //go to ball and push
        //done
















//        straight((float)20.0);
//        left(5);
//        runtime.reset();
//
//
//
//        mp1 = new PID(.001, 0, 0, .18, -.18, 1, -1, 20);
//
//        mp2 = new PID(.001, 0, 0, .18, -.18, 1, -1, 20);
//
//
//        pid = new PID(.02, 0, .015, .18, -.18, 1, -1, 20);
//
//
//        startTime = System.currentTimeMillis();



//        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        pid.setProcess(gyro.getIntegratedZValue());
//        mp1.setProcess(motor1.getCurrentPosition());
//        mp2.setProcess(motor3.getCurrentPosition());
//
//
//        currentTime = System.currentTimeMillis();
//        diff = currentTime - startTime;


        /*telemetry.addData("Encoder1", motor1.getCurrentPosition());
        telemetry.addData("Encoder2", motor2.getCurrentPosition());
        telemetry.addData("Encoder3", motor3.getCurrentPosition());
        telemetry.addData("Encoder4", motor4.getCurrentPosition());*/

        /*m1 = (gamepad1.right_stick_x);
        m2 = -m1;

        m1 += gamepad1.left_stick_y;
        m2 += gamepad1.left_stick_y;*/
//
//
//        if(task == 0)
//        {
//            m1 = mp1.getOutput();
//            m2 = mp2.getOutput();
//        }
//
//        if(task == 1)
//        {
//            m1 = pid.getOutput();
//            m2 = m1;
//        }
//
//        if(task == 2)
//        {
//            m1 = mp1.getOutput();
//            m2 = mp2.getOutput();
//        }
//
//        switch ((int)diff)
//        {
//            case 1://task1
//            {
//                mp1.setPoint(-1000);
//                mp2.setPoint(1000);
//                task = 0;
//
//                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            }
//
//            case 5001://task2
//            {
//                pid.setPoint(90);
//                task = 1;
//
//                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            case 10001://task3
//            {
//                mp1.setPoint(-2000);
//                mp2.setPoint(2000);
//                task = 2;
//
//                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            break;
//        }







//        motor1.setPower(m1);
//        motor2.setPower(m1);
//        motor3.setPower(m2);
//        motor4.setPower(m2);
    }

    public void straight(float distance) { // distance to move forward in inches

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setTargetPosition((int)((distance / CIRCUMFERENCE) * ENCODER));
        motor2.setTargetPosition((int)((distance / CIRCUMFERENCE) * ENCODER));
        motor3.setTargetPosition(-(int)((distance / CIRCUMFERENCE) * ENCODER));
        motor4.setTargetPosition(-(int)((distance / CIRCUMFERENCE) * ENCODER));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);
        motor4.setPower(1);
    }

    public void backwards(float distance) { // distance to move forward in inches

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setTargetPosition(-(int)((distance / CIRCUMFERENCE) * ENCODER));
        motor2.setTargetPosition(-(int)((distance / CIRCUMFERENCE) * ENCODER));
        motor3.setTargetPosition((int)((distance / CIRCUMFERENCE) * ENCODER));
        motor4.setTargetPosition((int)((distance / CIRCUMFERENCE) * ENCODER));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);
        motor4.setPower(1);
    }

    public void left(float distance) { // distance to move forward in inches

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setTargetPosition((int)((distance / CIRCUMFERENCE) * ENCODER));
        motor2.setTargetPosition((int)((distance / CIRCUMFERENCE) * ENCODER));
        motor3.setTargetPosition((int)((distance / CIRCUMFERENCE) * ENCODER));
        motor4.setTargetPosition((int)((distance / CIRCUMFERENCE) * ENCODER));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);
        motor4.setPower(1);
    }

    public void right(float distance) { // distance to move forward in inches

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setTargetPosition(-(int)((distance / CIRCUMFERENCE) * ENCODER));
        motor2.setTargetPosition(-(int)((distance / CIRCUMFERENCE) * ENCODER));
        motor3.setTargetPosition(-(int)((distance / CIRCUMFERENCE) * ENCODER));
        motor4.setTargetPosition(-(int)((distance / CIRCUMFERENCE) * ENCODER));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);
        motor4.setPower(1);
    }
}
