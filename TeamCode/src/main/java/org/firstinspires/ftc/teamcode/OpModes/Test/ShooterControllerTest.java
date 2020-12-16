package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;
import org.firstinspires.ftc.teamcode.Components.MainRobot;

@TeleOp
public class ShooterControllerTest extends LinearOpMode {

    MainRobot mainRobot;

    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        mainRobot.shooter.enableController();

        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();

        while(opModeIsActive()) {

            mainRobot.shooter.setpoint = Math.max(-1,
                    Math.min(1, mainRobot.shooter.setpoint + (0.005 * gamepad1.right_trigger)));

            telemetry.addData("Shooter Setpoint: ", mainRobot.shooter.setpoint);
            telemetry.addData("Shooter Power: ", mainRobot.shooter.getShooterPower());

        }

    }

}
