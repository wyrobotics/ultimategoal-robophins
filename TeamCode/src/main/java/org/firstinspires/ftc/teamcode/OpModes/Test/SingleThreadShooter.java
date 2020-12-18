package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

public class SingleThreadShooter extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        double integrator = 0;

        while(opModeIsActive()) {

            mainRobot.shooter.setpoint = Math.max(-1, Math.min(1,
                    mainRobot.shooter.setpoint + (0.005 * (gamepad1.right_trigger - gamepad1.left_trigger))));

            mainRobot.shooter.shooterController(integrator);

            telemetry.addData("Shooter setpoint: ", mainRobot.shooter.setpoint);

            telemetry.update();

        }

    }

}
