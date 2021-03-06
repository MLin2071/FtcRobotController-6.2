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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TouchSensorDrive", group="Linear Opmode")
public class TouchSensorDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1Drive = null;
    private DcMotor right1Drive = null;
    private DcMotor left2Drive =null;
    private DcMotor right2Drive = null;
    private DcMotor ramp1Motor = null;
    private DcMotor ramp2Motor = null;
    private DcMotor intakeMotor  = null;

    DigitalChannel digitalTouch;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        left1Drive  = hardwareMap.get(DcMotor.class, "left1");
        right1Drive = hardwareMap.get(DcMotor.class, "right1");
        left2Drive  = hardwareMap.get(DcMotor.class, "left2");
        right2Drive = hardwareMap.get(DcMotor.class, "right2");
        ramp1Motor = hardwareMap.get(DcMotor.class, "ramp1");
        ramp2Motor = hardwareMap.get(DcMotor.class, "ramp2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake" );



        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        left1Drive.setDirection(DcMotor.Direction.FORWARD);
        right1Drive.setDirection(DcMotor.Direction.REVERSE);
        left2Drive.setDirection(DcMotor.Direction.FORWARD);
        right2Drive.setDirection(DcMotor.Direction.REVERSE);
        ramp1Motor.setDirection(DcMotor.Direction.FORWARD);
        ramp2Motor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            double leftPower;
            double rightPower;


            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;



            left1Drive.setPower(leftPower);
            left2Drive.setPower(leftPower);
            right1Drive.setPower(rightPower);
            right2Drive.setPower(rightPower);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            if (digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
                intakeMotor.setPower(0);
                ramp1Motor.setPower(0);
                ramp2Motor.setPower(0);

            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
                intakeMotor.setPower(0.5);
                ramp1Motor.setPower(0.5);
                ramp2Motor.setPower(0.5);
            }

            telemetry.update();

            }

        }
    }


