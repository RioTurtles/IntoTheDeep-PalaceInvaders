package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoObservation_v1 extends LinearOpMode {

    public enum MoveStep{
        INIT;
    }

    MoveStep movestep = MoveStep.INIT;
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    Trajectory init;
    double maxVel, maxAccel;

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {

        }

    }
}
