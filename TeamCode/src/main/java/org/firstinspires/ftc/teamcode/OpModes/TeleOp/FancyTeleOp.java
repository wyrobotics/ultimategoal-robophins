package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.provider.Settings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Hardware.WobbleGoalArm;
import org.firstinspires.ftc.teamcode.Components.MainRobot;
import org.firstinspires.ftc.teamcode.Components.Software.GlobalPositioning;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp(group = "advanced")
public class FancyTeleOp extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

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
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(GlobalPositioning.poseMemory);

        mainRobot = new MainRobot(hardwareMap, telemetry);

        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();
        //mainRobot.shooter.startShooter();

        double integrator = 0;

        mainRobot.shooter.setpoint = 0;

        mainRobot.shooter.initPos();

        mainRobot.shooter.teleLastTime = System.currentTimeMillis();

        mainRobot.shooter.enableController();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

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

            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(targetBVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }

            mainRobot.deng(10);

        }

        mainRobot.shooter.disableController();
        mainRobot.shooter.shutdownShooter();
        mainRobot.hardwareThreadExecutor.shutdownExecutor();

    }
}