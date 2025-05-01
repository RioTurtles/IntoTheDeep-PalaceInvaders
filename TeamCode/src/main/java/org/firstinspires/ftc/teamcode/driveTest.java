package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class driveTest extends LinearOpMode { // TODO: WRITE RIGGING + SPECIMEN REWRITE

    @Override
    public void runOpMode() {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);

        Gamepad gamepad = new Gamepad();

        double vertical, horizontal, pivot, heading;

        robot.imu = hardwareMap.get(IMU.class, "imu");

        robot.frontLeft = hardwareMap.get(DcMotorEx.class, "motorFL");
        robot.frontRight = hardwareMap.get(DcMotorEx.class, "motorFR");
        robot.backLeft = hardwareMap.get(DcMotorEx.class, "motorBL");
        robot.backRight = hardwareMap.get(DcMotorEx.class, "motorBR");

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE); // TODO: CHECK DIRECTION
        robot.frontRight.setDirection(DcMotor.Direction.FORWARD); // TODO: CHECK DIRECTION
        robot.backLeft.setDirection(DcMotor.Direction.REVERSE); // TODO: CHECK DIRECTION
        robot.backRight.setDirection(DcMotor.Direction.FORWARD); // TODO: CHECK DIRECTION

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // TODO: CHECK DIRECTION
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD // TODO: CHECK DIRECTION
                        )
                )
        );

        waitForStart();

        while (opModeIsActive()) {

            gamepad.copy(gamepad1);

            vertical = gamepad.left_stick_y; horizontal = -gamepad.left_stick_x;
            pivot = -gamepad.right_stick_x; heading = robot.getIMUYaw();

            if (gamepad.touchpad) robot.resetIMU();

            drivetrain.remote(vertical, horizontal, pivot, heading);
            telemetry.addLine("For drive training I suppose ;-;");
            telemetry.update();

        }

    }

}
