package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class GetMotorPos extends LinearOpMode {
    double armPos;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                robot.setVSliderPos(Project1Hardware.VSliderPosition.DOWN,1.0);
            }
            if (gamepad1.right_bumper) {
                robot.setVSliderPos(Project1Hardware.VSliderPosition.HIGH_BASKET, 1.0);
            }

            telemetry.addData("sliderLPos", robot.vSliderL.getCurrentPosition());
            telemetry.addData("sliderRPos", robot.vSliderR.getCurrentPosition());
            telemetry.update();
        }

    }

}