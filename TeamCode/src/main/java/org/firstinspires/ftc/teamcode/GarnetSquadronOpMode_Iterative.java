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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Template: Iterative OpMode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class GarnetSquadronOpMode_Iterative extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    DcMotor shooter;
    DcMotor intake;
    double m1 = 0;
    double m2 = 0;
    double m3 = 0;
    double m4 = 0;
    ModernRoboticsI2cGyro gyro;
    ColorSensor color1, color2, color3;
    int color = 0;
    DeviceInterfaceModule dInterface;
    DcMotorController lModule;
    Servo servo;
    boolean turning;

    //PID variables
    PID pid;
    PID mp1;
    PID mp2;
    int task = 0;
    boolean set = false;

    //task times
    int task1 = 0;
    int task2 = 5000;
    int task3 = 10000;

    final static double DIAMETER = 4;
    final static int ENCODER = 1440;
    final static double CIRCUMFERENCE = Math.PI * DIAMETER;


    long currentTime, diff, startTime;


    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");

        servo = hardwareMap.servo.get("servo");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        dInterface = hardwareMap.deviceInterfaceModule.get("deviceInterfaceModule");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        color1 = hardwareMap.colorSensor.get("color1");
        color2 = hardwareMap.colorSensor.get("color2");
        color3 = hardwareMap.colorSensor.get("color3");

        gyro.calibrate();

        turning = true;

        try
        {
            Thread.sleep(3000);
        }
        catch(InterruptedException e)
        {
            telemetry.addData("Error", "could not calibrate gyro");
        }

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




//        servo.setPosition(90);
//
//        try
//        {
//            Thread.sleep(1000);
//        }
//        catch(InterruptedException e)
//        {
//            telemetry.addData("Error", "could not turn servo");
//        }
//
//        servo.setPosition(0);


        telemetry.addData("done", null);
    }


     //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop(){}

    @Override
    public void start()
    {
        motor4.setPower(-1.0);
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
    }


    @Override
    public void loop()
    {
        shooter.setPower(1);

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

        m1 = (gamepad1.right_stick_x);
        m2 = -m1;

        m1 += gamepad1.left_stick_y;
        m2 += gamepad1.left_stick_y;


        motor1.setPower(m1);
        motor2.setPower(m1);
        motor3.setPower(m2);
        motor4.setPower(m2);



        if(color1.red() > 10 && color1.green() > 10 && color1.blue() > 10)
        {
            color = 1;
            telemetry.addData("color", "white");
        }
        else if(color1.red() > color1.green() && color1.red() > color1.blue())
        {
            color = 2;
            telemetry.addData("color", "red");
        }
        else if(color1.blue() > color1.red() && color1.blue() > color1.green())
        {
            color = 3;
            telemetry.addData("color", "blue");
        }
        else
        {
            color = 0;
            telemetry.addData("color", "n/a");
        }



        if(gamepad2.right_trigger > 50)
        {
            shooter.setMaxSpeed(2800);
        }

        if(gamepad2.right_bumper)
        {
            servo.setPosition(90);
        }

        if(gamepad1.left_bumper)
        {
            intake.setPower(1);
        }








       // telemetry.addData("Angle", gyro.getIntegratedZValue());
        telemetry.addData("red", color1.red());
        telemetry.addData("green", color1.green());
        telemetry.addData("blue", color1.blue());
        telemetry.addData("m1", m1);
        telemetry.addData("m2", m2);
        telemetry.addData("m3", m3);
        telemetry.addData("MaxSpeed", motor4.getMaxSpeed());
        telemetry.addData("m4", motor4.getCurrentPosition());
        //telemetry.addData("gP", pid.getP());
        //telemetry.addData("gI", pid.getI());
        ///telemetry.addData("gD", pid.getD());
        //telemetry.addData("gPID", pid.getOutput());
        //telemetry.addData("EncLeftP", mp1.getP());
        //telemetry.addData("EncLeftI", mp1.getI());
        //telemetry.addData("EncLeftD", mp1.getD());
        //telemetry.addData("EncLeftPID", mp1.getOutput());
        //telemetry.addData("EncRightP", mp2.getP());
        //telemetry.addData("EncRightI", mp2.getI());
        //telemetry.addData("EncRightD", mp2.getD());
        //telemetry.addData("EncRightPID", mp2.getOutput());
        telemetry.addData("Task", task);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

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
