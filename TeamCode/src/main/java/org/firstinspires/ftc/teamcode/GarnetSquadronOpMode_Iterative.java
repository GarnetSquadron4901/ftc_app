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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    double m1;
    double m2;
    CompassSensor compass;
    DeviceInterfaceModule dInterface;
    boolean turning = false;


    double angle = 0;
    double diff = 0;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");


        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");


        dInterface = hardwareMap.deviceInterfaceModule.get("deviceInterfaceModule");


        compass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        runtime.reset();
    }

    @Override
    public void loop()
    {
        telemetry.addData("Status", "Running: " + runtime.toString());


        if(!gamepad1.left_bumper && !gamepad1.right_bumper && !turning)
        {
            angle = compass.getDirection();
            //telemetry.addData("Turn", "Neutral");
        }
        else if(gamepad1.left_bumper && !turning)
        {
            angle -= 90;
            turning = true;
            telemetry.addData("Turn", "Left");
        }
        else if(gamepad1.right_bumper && !turning)
        {
            angle += 90;
            turning = true;
            telemetry.addData("Turn", "Right");
        }

        if(angle >= 360)
            angle -= 360;
        else if(angle <= 0)
            angle += 360;

        if(compass.getDirection() < 180 && angle > compass.getDirection() && turning)
        {
            m1 = -1;
            m2 = 1;
        }
        else if(compass.getDirection() > 180 && angle < compass.getDirection() && turning)
        {
            m1 = 1;
            m1 = -1;
        }

        diff = angle - compass.getDirection();
        if(diff < 0)
            diff += 360;
        if(diff >= 360)
            diff -= 360;
        if(diff < 10 && turning)
        {
            turning = false;
            telemetry.addData("Stopped", "True");
        }


        telemetry.addData("Angle", compass.getDirection());
        telemetry.addData("Turning", turning);
        telemetry.addData("Diff", diff);
        telemetry.addData("Target", angle);


        /*m1 = (gamepad1.left_stick_y);
        m2 = -m1;

        m1 += gamepad1.right_stick_x;
        m2 += gamepad1.right_stick_x;*/






        m1 = 0;
        m2 = 0;


        m1 *= -1;
        m2 *= -1;

        motor1.setPower(m1);
        motor2.setPower(m1);
        motor3.setPower(m2 * -1);
        motor4.setPower(m2 * -1);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }
}
