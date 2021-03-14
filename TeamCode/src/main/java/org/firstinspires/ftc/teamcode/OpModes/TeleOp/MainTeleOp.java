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
    boolean leftTab;

    boolean shooterController = false;

    int shootIndex = 0;
    double speed = 0; //variable for shooter speed

    int macro = 710;
    double topGoalSpeed = 0.9;
    double powerShotSpeed = 0.8;

    double[] shooterSpeeds = new double[] {0, topGoalSpeed, powerShotSpeed};

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();
        //mainRobot.shooter.startShooter();

        double integrator = 0;

        mainRobot.shooter.setpoint = 0;

        mainRobot.shooter.initPos();

        mainRobot.shooter.teleLastTime = System.currentTimeMillis();

        mainRobot.shooter.enableController();

        while(opModeIsActive()) {

            if(gamepad1.left_trigger > 0) { mainRobot.drivebase.discOrtho(-this.gamepad1.left_stick_x * .2, this.gamepad1.left_stick_y * .2,
                    this.gamepad1.right_stick_x * .2); }
            else {mainRobot.drivebase.discOrtho(-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x);}
/*
            mainRobot.intake.intake(Math.max(gamepad1.right_trigger - gamepad1.left_trigger,
                    gamepad1.right_bumper ? 1 : -69));

            mainRobot.ramp.moveRamp(gamepad1.right_bumper ? 1 : 0);

            mainRobot.ramp.moveRamp(gamepad1.left_bumper ? -1 : 0); */

            //ANTHONY CODE ANTHONY CODE
            if(gamepad1.right_bumper) {
                mainRobot.ramp.moveRamp(1);
                mainRobot.intake.intake(1);
            } //set ramp and intake to go up if right bumper is down
            else if(gamepad1.right_trigger > 0) {
                mainRobot.ramp.moveRamp(-1);
                mainRobot.intake.intake(-1);
            } //if right bumper is not down and right trigger is compressed, set ramp and intake to down
            else {
                mainRobot.ramp.moveRamp(0);
                mainRobot.intake.intake(0);
            } //if neither right trigger nor right bumper are compressed, set ramp and intake to off

            /*
            if (gamepad1.dpad_down) {speed = .69;} //if dpad is down, set shooting speed to power shot speed
            else if(gamepad1.dpad_left || gamepad1.dpad_right) {speed = .72;} //medium speed
            else if(gamepad1.dpad_up) {speed = .8;} //if dpad is up and also not down, set shooting speed to top goal speed
            else if(gamepad1.x) {speed = 0;} //if neither dpad up nor dpad down are pressed, and x is pressed, set shooting speed to zero

            mainRobot.shooter.simpleShoot(speed); //set shooter motors to shooting speed as described above lulw git rekt documentation kekw
            */

            if (gamepad1.y) {mainRobot.jig(1);} //jiggle once
            //END ANTHONY CODE ANTHONY CODE

            //if(gamepad1.dpad_down) { mainRobot.shooter.setpoint = 800;} //power shot
            if(gamepad1.dpad_left || gamepad1.dpad_right) { mainRobot.shooter.setpoint = 810; }
            else if(gamepad1.dpad_up) { mainRobot.shooter.setpoint = 880; } //high goal
            else if(gamepad1.x) { mainRobot.shooter.setpoint = 0; }

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

            if(!leftTab && gamepad1.left_bumper) {
                leftTab = true;
                mainRobot.shooter.flick();
            } else if(leftTab && !gamepad1.left_bumper) {
                leftTab = false;
                mainRobot.shooter.unflick();
            }
            if(gamepad1.dpad_down) { mainRobot.drivebase.timedMovement(-.5, 0, 0, macro, 50); }
            if(gamepad2.dpad_up) { macro += 10; }
            if(gamepad2.dpad_down) { macro -= 10; }
            telemetry.addData("ms:", macro);


/*
            if(gamepad1.x && !xDown) {
                shootIndex += 1;
                mainRobot.shooter.simpleShoot(shooterSpeeds[shootIndex % 3]);
            } else if(xDown && !gamepad1.x) {
                xDown = false;
            }

            telemetry.addData("Shooter speed: ", shootIndex == 0 ? "Zero" :
                            (shootIndex == 1 ? "Top Goal" : "Power Shot"));
*/

            telemetry.update();
            mainRobot.deng(10);

        }


        mainRobot.shooter.disableController();
        mainRobot.shooter.shutdownShooter();
        mainRobot.hardwareThreadExecutor.shutdownExecutor();
        //mainRobot.shooter.shutdownShooter();

    }

}
