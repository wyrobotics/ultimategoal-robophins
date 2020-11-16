package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

import static org.firstinspires.ftc.teamcode.Components.Software.GlobalPositioning.robotTheta;
import static org.firstinspires.ftc.teamcode.Components.Software.GlobalPositioning.robotX;
import static org.firstinspires.ftc.teamcode.Components.Software.GlobalPositioning.robotY;

@TeleOp
public class SampleTeleOp extends LinearOpMode {

    MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        mainRobot.odometryTracker.startOdometry();

        while(opModeIsActive()) {

            mainRobot.drivebase.discOrtho(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y,
                    this.gamepad1.right_stick_x);

            mainRobot.intake.intake(gamepad1.left_trigger);

            mainRobot.shooter.simpleShoot(gamepad1.right_trigger > 0 ? 0.8 : 0);

            if(gamepad1.dpad_down) { mainRobot.wobbleGoalArm.down(); }
            else if(gamepad1.dpad_up) { mainRobot.wobbleGoalArm.lift(); }

            if(gamepad1.a) { mainRobot.wobbleGoalArm.grab(); }
            else if(gamepad1.b) {mainRobot.wobbleGoalArm.release(); }

            telemetry.addData("Robot X: ", robotX);
            telemetry.addData("Robot Y: ", robotY);
            telemetry.addData("Robot Heading: ", robotTheta);
            telemetry.update();

        }

        mainRobot.odometryTracker.shutdownOdometry();

    }

}
