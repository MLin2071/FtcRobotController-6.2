
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Encoder", group="Linear Opmode")

public class Encoder extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1Drive = null;
    private DcMotor right1Drive = null;
    private DcMotor left2Drive = null;
    private DcMotor right2Drive = null;


    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double WHEEL_DIAMETER = 10.16;
    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER * 3.1415);


    @Override
    public void runOpMode() {

        left1Drive = hardwareMap.get(DcMotor.class, "motor0");
        left1Drive.setDirection(DcMotor.Direction.REVERSE);

        left2Drive = hardwareMap.get(DcMotor.class, "motor1");
        left2Drive.setDirection(DcMotor.Direction.REVERSE);

        right1Drive = hardwareMap.get(DcMotor.class, "motor2");
        right1Drive.setDirection(DcMotor.Direction.FORWARD);

        right2Drive = hardwareMap.get(DcMotor.class, "motor3");
        right2Drive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        left1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() &&
                (left1Drive.getCurrentPosition() / COUNTS_PER_CM) < 100) {

            left1Drive.setPower(0.5);
            left2Drive.setPower(0.5);
            right1Drive.setPower(0.5);
            right2Drive.setPower(0.5);

        }


        left1Drive.setPower(0);
        left2Drive.setPower(0);
        right1Drive.setPower(0);
        right2Drive.setPower(0);


    }
}
