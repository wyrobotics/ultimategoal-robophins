package org.firstinspires.ftc.teamcode.OpModes.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

public class AutoMovementTest extends LinearOpMode {

    MainRobot mainRobot;

    double targetX;
    double targetY;
    double targetTheta;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();

        mainRobot.drivebase.gotoPos(targetX,targetY,targetTheta);

        mainRobot.hardwareThreadExecutor.shutdownExecutor();

    }

}
