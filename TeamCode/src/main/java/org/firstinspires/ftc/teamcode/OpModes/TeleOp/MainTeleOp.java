package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Hardware.GlobalConfig;
import org.firstinspires.ftc.teamcode.Components.Hardware.WobbleGoalArm;
import org.firstinspires.ftc.teamcode.Components.MainRobot;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    MainRobot mainRobot;

    boolean aDown;
    boolean bDown;
    boolean xDown;
    boolean rightTab;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();
        //mainRobot.shooter.startShooter();

        while(opModeIsActive()) {

            mainRobot.drivebase.discOrtho(this.gamepad1.right_stick_x, this.gamepad1.right_stick_y,
                    this.gamepad1.left_stick_x);

            mainRobot.intake.intake(Math.max(gamepad1.right_trigger - gamepad1.left_trigger,
                    gamepad1.right_bumper ? 1 : -69));

            mainRobot.ramp.moveRamp(Math.max(gamepad1.right_trigger - gamepad1.left_trigger,
                    gamepad1.right_bumper ? 1 : -69));

            if(gamepad1.dpad_up) {
                mainRobot.shooter.setpoint += 0.01;
                mainRobot.shooter.simpleShoot(mainRobot.shooter.setpoint);
            } else if(gamepad1.dpad_down) {
                mainRobot.shooter.setpoint -= 0.01;
                mainRobot.shooter.simpleShoot(mainRobot.shooter.setpoint);
            }

            if(!bDown && gamepad1.b) {
                bDown = true;
                if (mainRobot.wobbleGoalArm.mode == WobbleGoalArm.Mode.REST) {
                    mainRobot.wobbleGoalArm.lift();
                } else {
                    mainRobot.wobbleGoalArm.down();
                }
            } else if(bDown && !gamepad1.b) { bDown = false; }

            if(!aDown && gamepad1.a) {
                aDown = true;
                if (mainRobot.wobbleGoalArm.grabberMode == WobbleGoalArm.GrabberMode.OPEN) {
                    mainRobot.wobbleGoalArm.grab();
                } else {
                    mainRobot.wobbleGoalArm.release();
                }
            } else if(aDown && !gamepad1.a) { aDown = false; }

            if(!rightTab && gamepad1.right_bumper) {
                rightTab = true;
                if (mainRobot.shooter.flicked) {
                    mainRobot.shooter.unflick();
                } else {
                    mainRobot.shooter.flick();
                }
            } else if(rightTab && !gamepad1.right_bumper) { rightTab = false; }

        }

        telemetry.addData("Motor Power: ", mainRobot.shooter.setpoint);
        telemetry.update();

        mainRobot.hardwareThreadExecutor.shutdownExecutor();
        //mainRobot.shooter.shutdownShooter();

    }

}
