package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * ---------------------------- FTC DASHBOARD ------------------------------------
 * https://acmerobotics.github.io/ftc-dashboard/
 * To open the dashboard connect your laptop to the robot's wifi and access this address using a browser:
 * http://192.168.43.1:8080/dash
 *
 */
@Config
@Autonomous(name="FTC_Dashboard", group="Linear Opmode")
//@Disabled
public class DashBoard_ extends LinearOpMode {

    FtcDashboard dashboard;

    public static  double     kp = 0.00202674492; // kp=1/(704.86*constant) = constant = 0.7;
    public static  double     Target = 50;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while(opModeIsActive()){
            print(dashboardTelemetry);
        }

    }

    public void print(Telemetry dashboardTelemetry){

        dashboardTelemetry.addData("Target", Target);
        dashboardTelemetry.addData("Kp", kp);
        dashboardTelemetry.update();

    }

}