package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@TeleOp
public class SingleThreadShooter extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        double integrator = 0;

        mainRobot.shooter.setpoint = 2880;

        mainRobot.shooter.initPos();

        while(opModeIsActive()) {

            //mainRobot.shooter.setpoint = Math.max(-1, Math.min(1,
                    //mainRobot.shooter.setpoint + (0.005 * (gamepad1.right_trigger - gamepad1.left_trigger))));

            mainRobot.shooter.shooterController(integrator);

            telemetry.addData("RPS: ", mainRobot.shooter.getRPM() / 60 * 1000);

            telemetry.addData("Shooter setpoint: ", mainRobot.shooter.setpoint);

            telemetry.update();

            mainRobot.deng(10);

        }

    }

}
