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


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.Locale;


//@Config
@Autonomous(name="Tier 3", group="Linear Opmode")

public class Tier3 extends LinearOpMode {
//jhfyjtdjytdjgdjdjtd


    float hsvValues[] = {0F, 0F, 0F};

    final float values[] = hsvValues;

    final double SCALE_FACTOR = 255;


    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;
    Servo servo;
    double position = (MAX_POS - MIN_POS) / 2;

    FtcDashboard dashboard;
    BNO055IMU imu;
    Orientation angles;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    private DcMotor left1 = null;
    private DcMotor left2 = null;
    private DcMotor right1 = null;
    private DcMotor right2 = null;
    private ElapsedTime runtime = new ElapsedTime();

    public static double P_DRIVE_COEFF = 0.02;
    public static double Target = 500;


    static final double COUNTS_PER_MOTOR_REV = 704.86;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        servo = hardwareMap.get(Servo.class, "servo");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
        left1 = hardwareMap.get(DcMotor.class, "left_1");
        left2 = hardwareMap.get(DcMotor.class, "left_2");
        right1 = hardwareMap.get(DcMotor.class, "right_1");
        right2 = hardwareMap.get(DcMotor.class, "right_2");

        left1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);


        resetEncoders();

        waitForStart();
        gyroDrive(0.2, Target, 0, dashboardTelemetry);


    }

    public void gyroDrive(double speed, double distance, double angle, Telemetry dashboardTelemetry) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        if (opModeIsActive()) {

            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = left1.getCurrentPosition() + moveCounts;
            newRightTarget = right1.getCurrentPosition() + moveCounts;

            left1.setTargetPosition(newLeftTarget);
            left2.setTargetPosition(newLeftTarget);
            right1.setTargetPosition(newRightTarget);
            right2.setTargetPosition(newRightTarget);

            left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            while (opModeIsActive() &&
                    left1.isBusy() && right1.isBusy() && left2.isBusy() && right2.isBusy()) {

                getColour(); //////////////////////////////////////////////

                error = getErrorAngle(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                left1.setPower(leftSpeed);
                left2.setPower(leftSpeed);

                right1.setPower(rightSpeed);
                right2.setPower(rightSpeed);

                dashboardTelemetry.addData("Left Speed", leftSpeed);
                dashboardTelemetry.addData("Right Speed", rightSpeed);
                //print();

            }


            left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public double getCurrentPosition() {
        return Math.abs(left1.getCurrentPosition() + left2.getCurrentPosition()
                + right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
        // Math.abs returns the absolute value of an argument
    }

    public double getErrorAngle(double targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError;


        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void resetEncoders() {
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public String getColour() {

        String curColour= " ";

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);


        if (hsvValues[0] > 53 && hsvValues[0] < 70) {
            if (hsvValues[1] > 0.55 && hsvValues[1] < 0.8)
                if (hsvValues[2] > 150 && hsvValues[2] < 2300) {
                    telemetry.addLine("This could be orange");
                    curColour = "orange";
                    servo.setPosition(1);

                }
        }
        if (hsvValues[0] > 29.4 && hsvValues[0] < 50) {
            if (hsvValues[1] > 0.314 && hsvValues[1] < 0.638)
                if (hsvValues[2] > 70 && hsvValues[2] < 1578) {
                    telemetry.addLine("This could be red");
                    curColour = "red";
                    servo.setPosition(0);

                }
        }
        if (hsvValues[0] > 200 && hsvValues[0] < 220)
            if (hsvValues[1] > 0.6 && hsvValues[1] < 0.8)
                if (hsvValues[2] > 99 && hsvValues[2] < 1065) {

                    telemetry.addLine("This could be blue ");
                    curColour = "blue";
                    servo.setPosition(0.5);

                }


        telemetry.update();

                return curColour;
    }

    public void Servo () {


    }

        public void stopMotors(){
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);
        }


    }






















