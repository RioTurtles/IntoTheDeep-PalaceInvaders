package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Project1Hardware {
    DcMotorEx frontLeft, frontRight, backLeft, backRight, vSliderL, vSliderR;
    Servo intakeArm, intakePitch, claw, clawPitch, armL, armR, hSliderL, hSliderR;
    CRServo intake;
    ColorRangeSensor colorRangeSensor; // TODO
    IMU imu;
    HardwareMap hwmap;
    public Project1Hardware(HardwareMap hardwareMap) {
        hwmap = hardwareMap;
    }
    public void init(HardwareMap hardwareMap) {
        hwmap = hardwareMap;

        frontLeft = hardwareMap.get(DcMotorEx.class, "motorFL");
        frontRight = hardwareMap.get(DcMotorEx.class, "motorFR");
        backLeft = hardwareMap.get(DcMotorEx.class, "motorBL");
        backRight = hardwareMap.get(DcMotorEx.class, "motorBR");

        intake = hardwareMap.get(CRServo.class, "intake");
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        intakePitch = hardwareMap.get(Servo.class, "intakePitch");

        claw = hardwareMap.get(Servo.class, "claw");
        clawPitch = hardwareMap.get(Servo.class, "clawPitch");

        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");

        hSliderL = hardwareMap.get(Servo.class, "hSliderL");
        hSliderR = hardwareMap.get(Servo.class, "hSliderR");
        vSliderL = hardwareMap.get(DcMotorEx.class, "vSliderL");
        vSliderR = hardwareMap.get(DcMotorEx.class, "vSliderR");

        imu = hardwareMap.get(IMU.class, "imu");

        colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorRangeSensor");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE); // TODO: CHECK DIRECTION
        frontRight.setDirection(DcMotor.Direction.FORWARD); // TODO: CHECK DIRECTION
        backLeft.setDirection(DcMotor.Direction.REVERSE); // TODO: CHECK DIRECTION
        backRight.setDirection(DcMotor.Direction.FORWARD); // TODO: CHECK DIRECTION

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD); // TODO: CHECK DIRECTION
        intakeArm.setDirection(Servo.Direction.FORWARD); // TODO: CHECK DIRECTION
        intakePitch.setDirection(Servo.Direction.FORWARD); // TODO: CHECK DIRECTION

        claw.setDirection(Servo.Direction.FORWARD); // TODO: CHECK DIRECTION
        clawPitch.setDirection(Servo.Direction.FORWARD); // TODO: CHECK DIRECTION

        hSliderL.setDirection(Servo.Direction.FORWARD); // TODO: CHECK DIRECTION
        hSliderR.setDirection(Servo.Direction.FORWARD); // TODO: CHECK DIRECTION

        vSliderL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSliderR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSliderL.setDirection(DcMotor.Direction.FORWARD); // TODO: CHECK DIRECTION
        vSliderR.setDirection(DcMotor.Direction.FORWARD); // TODO: CHECK DIRECTION
        vSliderL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSliderR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset
        setIntakeMode(IntakeMode.STOP);
        setIntakeArmPos(IntakeArmPosition.UP);
        setIntakePitchPos(IntakePitchPosition.UP);
        setHSliderPos(HSliderPosition.IN);
        clawOpen();
        setClawPitchPos(ClawPitchPosition.TRANSFER);
        setVSliderPos(VSliderPosition.DOWN, 1.0);
        setArmPos(ArmPosition.TRANSFER);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP, // TODO: CHECK DIRECTION
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT // TODO: CHECK DIRECTION
                        )
                )
        );
    }

    public double getIMUYaw() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}
    public void resetIMU() {imu.resetYaw();}

    public enum IntakeMode {
        SUCK,
        SPIT,
        STOP
    }
    public void setIntakeMode(IntakeMode mode) {
        switch (mode) {
            case SUCK: intake.setPower(1); break;
            case SPIT: intake.setPower(-1); break;
            case STOP: intake.setPower(0); break;
        }
    }

    public enum IntakeArmPosition {
        UP,
        DOWN
    }

    public void setIntakeArmPos(IntakeArmPosition position) {
        switch (position) {
            case UP: intakePitch.setPosition(0); break;
            case DOWN: intakePitch.setPosition(1); break; // TODO: CHECK POSITION
        }
    }

    public enum IntakePitchPosition {
        UP,
        DOWN
    }

    public void setIntakePitchPos(IntakePitchPosition position) {
        switch (position) {
            case UP: intakePitch.setPosition(0); break;
            case DOWN: intakePitch.setPosition(1); break; // TODO: CHECK POSITION
        }
    }

    public void clawOpen() {claw.setPosition(0);}
    public void clawClose() {claw.setPosition(1);}

    public enum ClawPitchPosition {
        TRANSFER,
        HIGH_CHAMBER,
        HIGH_BASKET,
        SPECIMEN,
    }

    public void setClawPitchPos(ClawPitchPosition position) {
        switch (position) {
            case TRANSFER: clawPitch.setPosition(0); break;
            case HIGH_CHAMBER: clawPitch.setPosition(0.1); break;
            case HIGH_BASKET: clawPitch.setPosition(0.2); break;
            case SPECIMEN: clawPitch.setPosition(1); break;

        }
    }

    public enum ArmPosition {
        TRANSFER,
        HIGH_CHAMBER,
        HIGH_BASKET,
        SPECIMEN_UP,
        SPECIMEN
    }

    public void setArmPos(ArmPosition position) {
        switch (position) {
            case TRANSFER:
                armL.setPosition(0);
                armR.setPosition(0);
                break;
            case HIGH_CHAMBER:
                armL.setPosition(0.3); // TODO: CHECK POSITION
                armR.setPosition(0.3); // TODO: CHECK POSITION
                break;
            case HIGH_BASKET:
                armL.setPosition(0.6); // TODO: CHECK POSITION
                armR.setPosition(0.6); // TODO: CHECK POSITION
                break;
            case SPECIMEN_UP:
                armL.setPosition(0.9); // TODO: CHECK POSITION
                armR.setPosition(0.9); // TODO: CHECK POSITION
                break;
            case SPECIMEN:
                armL.setPosition(1); // TODO: CHECK POSITION
                armR.setPosition(1); // TODO: CHECK POSITION
                break;
        }
    }

    public enum VSliderPosition {
        DOWN,
        HIGH_CHAMBER,
        HIGH_BASKET
    }

    public void setVSliderPos(VSliderPosition position, double vSliderPower) {
        switch (position) {
            case DOWN:
                vSliderL.setTargetPosition(0);
                vSliderR.setTargetPosition(0);
                break;
            case HIGH_CHAMBER:
                vSliderL.setTargetPosition(500); // TODO: CHECK POSITION
                vSliderR.setTargetPosition(500); // TODO: CHECK POSITION
                break;
            case HIGH_BASKET:
                vSliderL.setTargetPosition(1000); // TODO: CHECK POSITION
                vSliderR.setTargetPosition(1000); // TODO: CHECK POSITION
                break;
        }
        vSliderL.setPower(vSliderPower);
        vSliderR.setPower(vSliderPower);
        vSliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean vSliderIsInPosition() {
        if ((vSliderL.getCurrentPosition() + vSliderR.getCurrentPosition()) / 2 >= (vSliderL.getTargetPosition() - 5)) {
            return true;
        } else return false;
    }

    public enum HSliderPosition {
        IN,
        OUT
    }

    public void setHSliderPos(HSliderPosition position) {
        switch (position) {
            case IN:
                hSliderL.setPosition(0);
                hSliderR.setPosition(0);
                break;
            case OUT:
                hSliderL.setPosition(1); // TODO: CHECK POSITION
                hSliderR.setPosition(1); // TODO: CHECK POSITION
                break;
        }
    }

    public enum SampleColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public SampleColor getSampleColor() {
        NormalizedRGBA color = colorRangeSensor.getNormalizedColors();
        double lightDetected = colorRangeSensor.getLightDetected();

        if (lightDetected >= 0.9) {
            if (color.red >= 0.9) {return SampleColor.RED;} else
            if (color.blue > 0.9) {return SampleColor.BLUE;} else
                return SampleColor.YELLOW;

        } else return SampleColor.UNKNOWN;

    }

}
