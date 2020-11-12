package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;
import org.firstinspires.ftc.teamcode.Components.MainRobot;
import static org.firstinspires.ftc.teamcode.Components.GlobalPositioning.*;

@TeleOp
public class MovementTest extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        mainRobot.odometryTracker.startOdometry();

        while(opModeIsActive()) {

            mainRobot.drivebase.discOrtho(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y,
                    this.gamepad1.right_stick_x);

            telemetry.addData("Robot X: ", robotX);
            telemetry.addData("Robot Y: ", robotY);
            telemetry.addData("Robot Heading: ", robotTheta);
            telemetry.update();

        }

        mainRobot.odometryTracker.shutdownOdometry();

    }

}
