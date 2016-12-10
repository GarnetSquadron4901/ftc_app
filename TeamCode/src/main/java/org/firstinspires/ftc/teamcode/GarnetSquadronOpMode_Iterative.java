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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
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
    double m1 = 0;
    double m2 = 0;
    CompassSensor compass;
    ModernRoboticsI2cGyro gyro;
    DeviceInterfaceModule dInterface;
    boolean turning;

    //PID variables
    PID pid;
    PID mp1;
    PID mp2;


    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");


        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");


        dInterface = hardwareMap.deviceInterfaceModule.get("deviceInterfaceModule");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

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


        //       while(motor1.isBusy() || motor2.isBusy() || motor3.isBusy() || motor4.isBusy()){} //wait for encoders to reset
        telemetry.addData("done", null);



        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


     //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop(){}
     //Code to run ONCE when the driver hits PLAY



    @Override
    public void start()
    {
        runtime.reset();

        pid = new PID(.02, 0, .015, .18, -.18, 1, -1, 20);
        pid.setPoint(90);

        mp1  = new PID(.001, 10, 0, .18, -.18, 1, -1, 20);
        mp1.setPoint(-1000);

        mp2  = new PID(.001, 10, 0, .18, -.18, 1, -1, 20);
        mp2.setPoint(1000);
    }


    @Override
    public void loop()
    {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pid.setProcess(gyro.getIntegratedZValue());

        telemetry.addData("Encoder1", motor1.getCurrentPosition());
        telemetry.addData("Encoder2", motor2.getCurrentPosition());
        telemetry.addData("Encoder3", motor3.getCurrentPosition());
        telemetry.addData("Encoder4", motor4.getCurrentPosition());

        mp1.setProcess(motor1.getCurrentPosition());
        m1 = mp1.getOutput();

        mp2.setProcess(motor3.getCurrentPosition());
        m2 = mp2.getOutput();



//        motor1.setTargetPosition(1000);
//        motor2.setTargetPosition(1000);
//        motor3.setTargetPosition(1000);
//        motor4.setTargetPosition(1000);

        /*m1 = (gamepad1.right_stick_x);
        m2 = -m1;

        m1 += gamepad1.left_stick_y;
        m2 += gamepad1.left_stick_y;*/





        /*m1 = pid.getOutput();
        m2 = pid.getOutput();

        if(Math.abs(m1) < .05)
            m1 = 0;
        if(Math.abs(m2) < .05)
            m2 = 0;*/



        motor1.setPower(m1);
        motor2.setPower(m1);
        motor3.setPower(m2);
        motor4.setPower(m2);



        telemetry.addData("Angle", gyro.getIntegratedZValue());
        telemetry.addData("m1", m1);
        telemetry.addData("m2", m2);
        telemetry.addData("gP", pid.getP());
        telemetry.addData("gI", pid.getI());
        telemetry.addData("gD", pid.getD());
        telemetry.addData("gPID", pid.getOutput());
        telemetry.addData("EncLeftP", mp1.getP());
        telemetry.addData("EncLeftI", mp1.getI());
        telemetry.addData("EncLeftD", mp1.getD());
        telemetry.addData("EncLeftPID", mp1.getOutput());
        telemetry.addData("EncRightP", mp2.getP());
        telemetry.addData("EncRightI", mp2.getI());
        telemetry.addData("EncRightD", mp2.getD());
        telemetry.addData("EncRightPID", mp2.getOutput());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }
}
