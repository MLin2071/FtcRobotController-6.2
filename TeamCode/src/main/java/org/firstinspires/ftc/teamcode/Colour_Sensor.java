package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "Colour sensor", group = "Sensor")

public class Colour_Sensor extends LinearOpMode {


    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    private DcMotor leftfrontDrive;
    private DcMotor leftbackDrive;
    private DcMotor rightfrontDrive;
    private DcMotor rightbackDrive;


    @Override
    public void runOpMode() {

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
        leftfrontDrive = hardwareMap.get(DcMotor.class, "left_front_Drive");
        leftbackDrive = hardwareMap.get(DcMotor.class,"left_back_Drive");
        rightfrontDrive = hardwareMap.get(DcMotor.class,"right_front_Drive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "right_back_Drive");

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;



        waitForStart();


        while (opModeIsActive()) {


            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            if(sensorColor.red()<100){

            }
            telemetry.update();
        }

    }
}