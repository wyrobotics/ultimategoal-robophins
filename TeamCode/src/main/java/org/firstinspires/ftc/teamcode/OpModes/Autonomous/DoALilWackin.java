package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@Autonomous
public class DoALilWackin extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        mainRobot.wacker.upperWacker();

        mainRobot.deng(2000);

        mainRobot.wacker.lowerWacker();

        mainRobot.deng(2000);

        mainRobot.wacker.upperWacker();

    }

}
