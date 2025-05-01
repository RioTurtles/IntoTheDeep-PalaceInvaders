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
//    ColorRangeSensor colorRangeSensor; // TODO
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

//        colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorRangeSensor");

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

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeArm.setDirection(Servo.Direction.FORWARD);
        intakePitch.setDirection(Servo.Direction.REVERSE);

        claw.setDirection(Servo.Direction.FORWARD);
        clawPitch.setDirection(Servo.Direction.REVERSE);

        armL.setDirection(Servo.Direction.REVERSE);
        armR.setDirection(Servo.Direction.REVERSE);

        hSliderL.setDirection(Servo.Direction.FORWARD);
        hSliderR.setDirection(Servo.Direction.REVERSE);

        vSliderL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSliderR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSliderL.setDirection(DcMotor.Direction.FORWARD); // TODO: CHECK DIRECTION
        vSliderR.setDirection(DcMotor.Direction.REVERSE); // TODO: CHECK DIRECTION
        vSliderL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        vSliderR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        //Reset
//        setIntakeMode(IntakeMode.STOP);
//        setIntakeArmPos(IntakeArmPosition.INIT);
//        setIntakePitchPos(IntakePitchPosition.INIT);
//        setHSliderPos(HSliderPosition.IN);
//        clawOpen();
//        setClawPitchPos(ClawPitchPosition.TRANSFER);
//        setVSliderPos(VSliderPosition.DOWN, 1.0);
//        setArmPos(ArmPosition.INIT);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
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
        INIT,
        TRANSFER,
        UP,
        RELEASE,
        DOWN
    }

    public void setIntakeArmPos(IntakeArmPosition position) {
        switch (position) {
            case INIT: intakeArm.setPosition(0); break;
            case TRANSFER: intakeArm.setPosition(0.05); break;
            case UP: intakeArm.setPosition(0.33); break;
            case RELEASE: intakeArm.setPosition(0.48); break;
            case DOWN: intakeArm.setPosition(0.53); break;
        }
    }

    public enum IntakePitchPosition {
        INIT,
        FLAT,
        TRANSFER,
        INTAKE,
        RELEASE
    }

    public void setIntakePitchPos(IntakePitchPosition position) {
        switch (position) {
            case INIT: intakePitch.setPosition(0); break;
            case FLAT: intakePitch.setPosition(0.25); break;
            case TRANSFER: intakePitch.setPosition(0); break;
            case INTAKE: intakePitch.setPosition(0.29); break;
            case RELEASE: intakePitch.setPosition(0.33); break;
        }
    }

    public void clawCompleteOpen() {claw.setPosition(0);}

    public void clawOpen() {claw.setPosition(0.15);}
    public void clawClose() {claw.setPosition(0.27);}

    public enum ClawPitchPosition {
        TRANSFER,
        HIGH_CHAMBER_READY,
        HIGH_CHAMBER,
        HIGH_BASKET,
        SPECIMEN,
    }

    public void setClawPitchPos(ClawPitchPosition position) {
        switch (position) {
            case TRANSFER: clawPitch.setPosition(0.009); break;
            case HIGH_CHAMBER_READY: clawPitch.setPosition(0.1); break; // TODO: CHECK POSITION
            case HIGH_CHAMBER: clawPitch.setPosition(0); break; // CHECK POSITION
            case HIGH_BASKET: clawPitch.setPosition(0.3); break;
            case SPECIMEN: clawPitch.setPosition(0.15); break;
        }
    }

    public enum ArmPosition {
        INIT,
        TRANSFER,
        HIGH_BASKET,
        SPECIMEN_UP,
        SPECIMEN
    }

    public void setArmPos(ArmPosition position) {
        switch (position) {
            case INIT:
                armL.setPosition(0);
                armR.setPosition(0);
                break;
            case TRANSFER:
                armL.setPosition(0.029);
                armR.setPosition(0.029);
                break;
            case HIGH_BASKET:
                armL.setPosition(0.33);
                armR.setPosition(0.33);
                break;
            case SPECIMEN_UP:
                armL.setPosition(0.48);
                armR.setPosition(0.48);
                break;
            case SPECIMEN:
                armL.setPosition(0.53);
                armR.setPosition(0.53);
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
                vSliderL.setTargetPosition(3000); // TODO: CHECK POSITION
                vSliderR.setTargetPosition(3000); // TODO: CHECK POSITION
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
    public enum HSliderState {
        IN,
        OUT_1,
        OUT_2,
        OUT_3
    }
    HSliderState hSliderState = HSliderState.IN;

    public void setHSliderPos(HSliderPosition position) { // TODO
        switch (position) {
            case IN:
                hSliderL.setPosition(0);
                hSliderR.setPosition(0);
                hSliderState = HSliderState.OUT_1;

                break;
            case OUT:
                switch (hSliderState) {
                    case OUT_1:
                        hSliderL.setPosition(0.11);
                        hSliderR.setPosition(0.11);
                        hSliderState = HSliderState.OUT_2;

                        break;
                    case OUT_2:
                        hSliderL.setPosition(0.22);
                        hSliderR.setPosition(0.22);
                        hSliderState = HSliderState.OUT_3;

                        break;
                    case OUT_3:
                        hSliderL.setPosition(0.33);
                        hSliderR.setPosition(0.33);

                        break;
                }

                break;
        }

    }

    public enum SampleColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

//    public SampleColor getSampleColor() {
//        NormalizedRGBA color = colorRangeSensor.getNormalizedColors();
//        double lightDetected = colorRangeSensor.getLightDetected();
//
//        if (lightDetected >= 0.9) { // TODO: CHECK DETECTION
//            if (color.red >= 0.9) {return SampleColor.RED;} else
//            if (color.blue > 0.9) {return SampleColor.BLUE;} else
//                return SampleColor.YELLOW;
//
//        } else return SampleColor.UNKNOWN;
//
//    }

}
