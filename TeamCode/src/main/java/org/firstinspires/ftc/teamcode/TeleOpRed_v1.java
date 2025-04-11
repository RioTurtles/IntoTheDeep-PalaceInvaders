package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Project1Hardware.HSliderPosition;
import org.firstinspires.ftc.teamcode.Project1Hardware.IntakeMode;
import org.firstinspires.ftc.teamcode.Project1Hardware.IntakeArmPosition;
import org.firstinspires.ftc.teamcode.Project1Hardware.IntakePitchPosition;
import org.firstinspires.ftc.teamcode.Project1Hardware.ArmPosition;
import org.firstinspires.ftc.teamcode.Project1Hardware.ClawPitchPosition;
import org.firstinspires.ftc.teamcode.Project1Hardware.VSliderPosition;
import org.firstinspires.ftc.teamcode.Project1Hardware.SampleColor;

import java.util.Objects;

@TeleOp
public class TeleOpRed_v1 extends LinearOpMode { // TODO: WRITE RIGGING

    double vSliderPower = 1.0;

    public enum State {
        INIT,
        SAMPLE_INTAKE,
        SAMPLE_INTAKE_DOWN,
        SUCKED_SAMPLE,
        RELEASE_SAMPLE,
        TRANSFER,
        HIGH_BASKET_1,
        HIGH_BASKET_2,
        HIGH_BASKET_3,
        SPECIMEN_INTAKE,
        GRABBED_SPECIMEN,
        PREPARE_CHAMBER_1,
        PREPARE_CHAMBER_2,
        HIGH_CHAMBER_1,
        HIGH_CHAMBER_2,
        FAILSAFE

    }

    @Override
    public void runOpMode() {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);
        robot.init(hardwareMap);

        State state = State.INIT;

        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        ElapsedTime loopTime = new ElapsedTime();
        PIDController headingController = new PIDController(0.55, 0.001, 0);
        Double autoAlignTarget;

        double vertical, horizontal, pivot, heading;
        boolean isSpecimen = true;
        boolean isBasket = false;

        waitForStart();

        while (opModeIsActive()) {

            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);

            vertical = gamepad.left_stick_y; horizontal = -gamepad.left_stick_x;
            pivot = -gamepad.right_stick_x; heading = robot.getIMUYaw();

            boolean lb = gamepad.left_bumper && !lastGamepad.left_bumper;
            boolean rb = gamepad.right_bumper && !lastGamepad.right_bumper;
            boolean lt = (gamepad.left_trigger > 0) && !(lastGamepad.left_trigger > 0);
            boolean rt = (gamepad.right_trigger > 0) && !(lastGamepad.right_trigger > 0);

            switch (state) {
                case INIT:
                    robot.setIntakeMode(IntakeMode.STOP);
                    robot.setIntakePitchPos(IntakePitchPosition.UP);
                    robot.setIntakeArmPos(IntakeArmPosition.UP);
                    robot.setHSliderPos(HSliderPosition.IN);

                    robot.clawOpen();
                    robot.setClawPitchPos(ClawPitchPosition.TRANSFER);
                    robot.setArmPos(ArmPosition.TRANSFER);
                    robot.setVSliderPos(VSliderPosition.DOWN, vSliderPower);

                    if (rb) {
                        state = State.SAMPLE_INTAKE;
                    }

                    break;
                case SAMPLE_INTAKE:
                    robot.setIntakePitchPos(IntakePitchPosition.DOWN);

                    if (lt) {robot.setHSliderPos(HSliderPosition.IN);}
                    if (rt) {robot.setHSliderPos(HSliderPosition.OUT);}

                    if (rb) {
                        robot.setIntakeArmPos(IntakeArmPosition.DOWN);
                        state = State.SAMPLE_INTAKE_DOWN;
                    }
                    if (lb) {
                        robot.setIntakeMode(IntakeMode.STOP);
                        robot.setIntakePitchPos(IntakePitchPosition.UP);
                        robot.setIntakeArmPos(IntakeArmPosition.UP);
                        state = State.INIT;
                    }

                    if (gamepad.square || operator.square) {isSpecimen = false;}
                    if (gamepad.circle || operator.circle) {isSpecimen = true;}
                    if (gamepad.cross || operator.cross) {isBasket = false;}
                    if (gamepad.triangle || operator.triangle) {isBasket = true;}

                    if (isSpecimen) {state = State.SPECIMEN_INTAKE;}

                    break;
                case SAMPLE_INTAKE_DOWN:
                    if (!isBasket) {
                        if (robot.getSampleColor() == SampleColor.BLUE || robot.getSampleColor() == SampleColor.YELLOW) {
                            robot.setIntakeMode(IntakeMode.SPIT);
                        } else {
                            robot.setIntakeMode(IntakeMode.SUCK);
                        }
                    } else {
                        robot.setIntakeMode(IntakeMode.SUCK);
                    }

                    if (lt) {robot.setHSliderPos(HSliderPosition.IN);}
                    if (rt) {robot.setHSliderPos(HSliderPosition.OUT);}

                    if (rb) {state = State.SUCKED_SAMPLE;}
                    if (lb) {state = State.SAMPLE_INTAKE;}

                    if (gamepad.cross || operator.cross) {isBasket = false;}
                    if (gamepad.triangle || operator.triangle) {isBasket = true;}

                    break;
                case SUCKED_SAMPLE:
                    robot.setIntakePitchPos(IntakePitchPosition.DOWN);
                    robot.setIntakeMode(IntakeMode.STOP);
                    robot.setIntakeArmPos(IntakeArmPosition.UP);

                    if (lt) {robot.setHSliderPos(HSliderPosition.IN);}
                    if (rt) {robot.setHSliderPos(HSliderPosition.OUT);}

                    if (rb) {
                        if (isBasket) {
                            state = State.TRANSFER;
                        } else {
                            state = State.RELEASE_SAMPLE;
                        }
                    }
                    if (lb) {robot.setIntakeArmPos(IntakeArmPosition.DOWN); state = State.SAMPLE_INTAKE_DOWN;}

                    if (gamepad.cross || operator.cross) {isBasket = false;}
                    if (gamepad.triangle || operator.triangle) {isBasket = true;}

                    break;
                case RELEASE_SAMPLE:
                    robot.setIntakeArmPos(IntakeArmPosition.DOWN);

                    if (lt) {robot.setHSliderPos(HSliderPosition.IN);}
                    if (rt) {robot.setHSliderPos(HSliderPosition.OUT);}

                    if (rb) {
                        if (robot.getSampleColor() == SampleColor.UNKNOWN) {
                            state = State.INIT;
                        } else {
                            robot.setIntakeMode(IntakeMode.SPIT);
                        }
                    }
                    if (lb) {state = State.SUCKED_SAMPLE;}

                    if (gamepad.square || operator.square) {isSpecimen = false;}
                    if (gamepad.circle || operator.circle) {isSpecimen = true;}
                    if (gamepad.cross || operator.cross) {isBasket = false;}
                    if (gamepad.triangle || operator.triangle) {isBasket = true;}

                    if (isSpecimen) {state = State.SPECIMEN_INTAKE;}

                    break;
                case TRANSFER: // TODO
                    robot.clawOpen();
                    robot.setIntakePitchPos(IntakePitchPosition.UP);
                    robot.setHSliderPos(HSliderPosition.IN);

                    if (rb) {state = State.HIGH_BASKET_1;}
                    if (lb) {state = State.SUCKED_SAMPLE;}

                    if (gamepad.cross || operator.cross) {isBasket = false;}
                    if (gamepad.triangle || operator.triangle) {isBasket = true;}

                    break;
                case HIGH_BASKET_1: // TODO
                    robot.clawClose();

                    if (rb) {state = State.HIGH_BASKET_2;}
                    if (lb) {state = State.TRANSFER;}

                    if (gamepad.cross || operator.cross) {isBasket = false;}
                    if (gamepad.triangle || operator.triangle) {isBasket = true;}

                    break;
                case HIGH_BASKET_2: // TODO
                    robot.clawClose();
                    robot.setVSliderPos(VSliderPosition.HIGH_BASKET, vSliderPower);
                    robot.setArmPos(ArmPosition.HIGH_BASKET);

                    if (rb) {state = State.HIGH_BASKET_3;}
                    if (lb) {robot.clawOpen(); state = State.INIT;}

                    if (gamepad.cross || operator.cross) {isBasket = false;}
                    if (gamepad.triangle || operator.triangle) {isBasket = true;}

                    break;
                case HIGH_BASKET_3:
                    robot.clawOpen();

                    if (rb) {state = State.INIT;}
                    if (lb) {state = State.HIGH_BASKET_3;}

                    if (gamepad.cross || operator.cross) {isBasket = false;}
                    if (gamepad.triangle || operator.triangle) {isBasket = true;}

                    break;
                case SPECIMEN_INTAKE:
                    robot.setIntakeMode(IntakeMode.STOP);
                    robot.setIntakePitchPos(IntakePitchPosition.UP);
                    robot.setIntakeArmPos(IntakeArmPosition.UP);
                    robot.setHSliderPos(HSliderPosition.IN);

                    robot.clawOpen();
                    robot.setArmPos(ArmPosition.SPECIMEN);
                    robot.setClawPitchPos(ClawPitchPosition.SPECIMEN);

                    if (rb) {state = State.GRABBED_SPECIMEN;}
                    if (lb) {state = State.INIT;}

                    if (gamepad.square || operator.square) {isSpecimen = false;}
                    if (gamepad.circle || operator.circle) {isSpecimen = true;}
                    if (gamepad.cross || operator.cross) {isBasket = false;}
                    if (gamepad.triangle || operator.triangle) {isBasket = true;}

                    if (!isSpecimen) {state = State.SAMPLE_INTAKE;}

                    break;
                case GRABBED_SPECIMEN:
                    robot.clawClose();
                    robot.setArmPos(ArmPosition.SPECIMEN);

                    if (rb) {state = State.PREPARE_CHAMBER_1;}
                    if (lb) {state = State.SPECIMEN_INTAKE;}

                    break;
                case PREPARE_CHAMBER_1:
                    robot.setArmPos(ArmPosition.SPECIMEN_UP);

                    if (rb) {state = State.PREPARE_CHAMBER_2;}
                    if (lb) {state = State.GRABBED_SPECIMEN;}

                    break;
                case PREPARE_CHAMBER_2:
                    robot.setVSliderPos(VSliderPosition.DOWN, vSliderPower);
                    robot.setArmPos(ArmPosition.HIGH_CHAMBER);

                    if (rb) {state = State.HIGH_CHAMBER_1;}
                    if (lb) {state = State.PREPARE_CHAMBER_1;}

                    break;
                case HIGH_CHAMBER_1:
                    robot.setVSliderPos(VSliderPosition.HIGH_CHAMBER, vSliderPower);

                    if (rb) {state = State.HIGH_CHAMBER_2;}
                    if (lb) {state = State.PREPARE_CHAMBER_2;}

                    break;
                case HIGH_CHAMBER_2:
                    robot.clawOpen();
                    robot.setVSliderPos(VSliderPosition.DOWN, vSliderPower);
                    state = State.SPECIMEN_INTAKE;

                    break;
            }

//            if (operator.touchpad && !lastOperator.touchpad) { // TODO
//                if (state == state.FAILSAFE) {
//                    state = State.INIT;
//                } else {
//                    state = State.FAILSAFE;
//                }
//            }

            if (operator.right_trigger > 0) autoAlignTarget = 0.0; // High basket allignment TODO
            else if (operator.left_bumper) autoAlignTarget = 0.0;  // Submersible or High chamber alignment TODO
            else if (operator.right_bumper) autoAlignTarget = 0.0;  // Observation zone alignment TODO
            else autoAlignTarget = null;

            if (Objects.nonNull(autoAlignTarget)) {
                assert autoAlignTarget != null;

                double current = Math.toDegrees(heading);
                double smallerAngle = Math.min(
                        Math.abs(current - autoAlignTarget),
                        360 - Math.abs(current - autoAlignTarget)
                );

                double resultant1 = current - smallerAngle;
                if (resultant1 <= -180) resultant1 += 360;
                double resultant2 = current + smallerAngle;
                if (resultant2 > 180) resultant2 -= 360;

                if (resultant1 == autoAlignTarget) pivot = Math.toRadians(smallerAngle);
                else if (resultant2 == autoAlignTarget) pivot = Math.toRadians(-smallerAngle);

                pivot = headingController.calculate(0, pivot, loopTime.seconds());

                vertical *= 0.8;
                horizontal *= 0.8;
            } else headingController.reset();

            if (gamepad.touchpad) robot.resetIMU();
            drivetrain.remote(vertical, horizontal, pivot, heading);
            telemetry.addData("State", state);
            telemetry.update();
            loopTime.reset();

        }

    }

}
