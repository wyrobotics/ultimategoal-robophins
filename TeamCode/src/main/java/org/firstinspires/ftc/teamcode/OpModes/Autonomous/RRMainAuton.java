package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RRMainRobot;
import org.firstinspires.ftc.teamcode.Components.Software.Pipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Arrays;

@Autonomous
public class RRMainAuton extends LinearOpMode {

    RRMainRobot mainRobot;

    OpenCvInternalCamera phoneCam;

    Pipeline pipeline = new Pipeline();

    int stackHeight(double area) { return (area < 100) ? 0 : ((area > 450) ? 2 : 1); }

    @Override
    public void runOpMode() {

        mainRobot = new RRMainRobot(hardwareMap, telemetry);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        phoneCam.setPipeline(pipeline);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        //phoneCam.setFlashlightEnabled(true);


        mainRobot.wobbleGoalArm.grab();

        mainRobot.shooter.setpoint = 0;

        mainRobot.shooter.initPos();

        mainRobot.shooter.enableController();


        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(180));


        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();

        telemetry.addData("Stack Height:", stackHeight(pipeline.getContourArea()));
        int height = stackHeight(pipeline.getContourArea());
        telemetry.update();

        phoneCam.stopStreaming();

        AutonTrajectories autonTrajectories = new AutonTrajectories(height);

        mainRobot.drive.setPoseEstimate(autonTrajectories.initPose);

        mainRobot.drive.followTrajectory(autonTrajectories.dropWobble);

        mainRobot.deng(200);

        mainRobot.wobbleGoalArm.lift();
        mainRobot.deng(1000);
        mainRobot.wobbleGoalArm.release();
        mainRobot.deng(500);
        mainRobot.wobbleGoalArm.down();
        mainRobot.deng(500);

        mainRobot.drive.followTrajectory(autonTrajectories.toPowerShot);

        mainRobot.deng(200);

        telemetry.addData("Heading: ", mainRobot.drive.getExternalHeading());
        telemetry.update();

        mainRobot.shooter.setpoint = 720;

        mainRobot.deng(2000);
        mainRobot.shooter.flick();
        mainRobot.deng(500);
        mainRobot.shooter.unflick();
        mainRobot.deng(500);

        mainRobot.drive.followTrajectory(autonTrajectories.powerShotStrafeOne);

        mainRobot.deng(500);
        mainRobot.shooter.flick();
        mainRobot.deng(500);
        mainRobot.shooter.unflick();
        mainRobot.deng(500);

        mainRobot.drive.followTrajectory(autonTrajectories.powerShotStrafeTwo);

        mainRobot.deng(500);
        mainRobot.shooter.flick();
        mainRobot.deng(500);
        mainRobot.shooter.unflick();
        mainRobot.deng(500);

        mainRobot.shooter.setpoint = 0;
        mainRobot.shooter.disableController();
        mainRobot.shooter.shutdownShooter();
        mainRobot.hardwareThreadExecutor.shutdownExecutor();
        mainRobot.shooter.simpleShoot(0);

        mainRobot.drive.followTrajectory(autonTrajectories.getSecondWobble);

        mainRobot.wobbleGoalArm.lift();
        mainRobot.deng(500);

        mainRobot.drive.followTrajectory(autonTrajectories.grabSecondWobble);

        mainRobot.deng(2500);
        mainRobot.wobbleGoalArm.grab();
        mainRobot.deng(1000);
        mainRobot.wobbleGoalArm.down();
        mainRobot.deng(500);

        mainRobot.drive.followTrajectory(autonTrajectories.dropSecondWobble);

        mainRobot.deng(500);
        mainRobot.wobbleGoalArm.lift();
        mainRobot.deng(1000);
        mainRobot.wobbleGoalArm.release();
        mainRobot.deng(500);
        mainRobot.wobbleGoalArm.down();

    }

    private class AutonTrajectories {

        Trajectory dropWobble;
        Trajectory toPowerShot;
        Trajectory powerShotStrafeOne;
        Trajectory powerShotStrafeTwo;
        Trajectory toLine;

        Trajectory getSecondWobble;
        Trajectory grabSecondWobble;
        Trajectory dropSecondWobble;
        Trajectory toLineFromWobble;

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(180));

        public AutonTrajectories(int stackHeight) {

            switch(stackHeight) {

                //0 RING STACK
                case 0:

                    dropWobble = mainRobot.drive.trajectoryBuilder(initPose)
                            .strafeLeft(5)
                            .splineToConstantHeading(new Vector2d(60, -13), Math.toRadians(180))
                            .build();

                    toPowerShot = mainRobot.drive.trajectoryBuilder(dropWobble.end())
                            .lineToLinearHeading(new Pose2d(62,12,0))
                            .build();

                    powerShotStrafeOne = mainRobot.drive.trajectoryBuilder(toPowerShot.end())
                            .strafeLeft(7)
                            .build();

                    powerShotStrafeTwo = mainRobot.drive.trajectoryBuilder(powerShotStrafeOne.end())
                            .strafeLeft(7)
                            .build();

                    getSecondWobble = mainRobot.drive.trajectoryBuilder(powerShotStrafeTwo.end())
                            .lineToLinearHeading(new Pose2d(41, 21, Math.toRadians(20)))
                            .build();

                    grabSecondWobble = mainRobot.drive.trajectoryBuilder(getSecondWobble.end())
                            .back(6,
                                    new MinVelocityConstraint(
                                            Arrays.asList(
                                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                    new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                            )
                                    ),
                                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

                    dropSecondWobble = mainRobot.drive.trajectoryBuilder(grabSecondWobble.end())
                            .lineToLinearHeading(new Pose2d(68, 0, Math.toRadians(90)))
                            .build();

                    toLineFromWobble = mainRobot.drive.trajectoryBuilder(dropSecondWobble.end())
                            .back(6)
                            .build();

                    break;

                //1 RING STACK
                case 1:

                    dropWobble = mainRobot.drive.trajectoryBuilder(initPose)
                            .strafeLeft(5)
                            .splineToConstantHeading(new Vector2d(60,-13), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(84,0), Math.toRadians(180))
                            .build();

                    toPowerShot = mainRobot.drive.trajectoryBuilder(dropWobble.end())
                            .lineToLinearHeading(new Pose2d(62,14,0))
                            .build();

                    powerShotStrafeOne = mainRobot.drive.trajectoryBuilder(toPowerShot.end())
                            .strafeLeft(5)
                            .build();

                    powerShotStrafeTwo = mainRobot.drive.trajectoryBuilder(powerShotStrafeOne.end())
                            .strafeLeft(7)
                            .build();

                    getSecondWobble = mainRobot.drive.trajectoryBuilder(powerShotStrafeTwo.end())
                            .lineToLinearHeading(new Pose2d(44, 20, Math.toRadians(20)))
                            .build();

                    grabSecondWobble = mainRobot.drive.trajectoryBuilder(getSecondWobble.end())
                            .back(10,
                                    new MinVelocityConstraint(
                                            Arrays.asList(
                                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                    new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                            )
                                    ),
                                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

                    dropSecondWobble = mainRobot.drive.trajectoryBuilder(grabSecondWobble.end())
                            .lineToLinearHeading(new Pose2d(68, 0, Math.toRadians(180)))
                            .build();

                    toLineFromWobble = mainRobot.drive.trajectoryBuilder(dropSecondWobble.end())
                            .back(5)
                            .build();

                    break;

                //4 RING STACK
                case 2:

                    dropWobble = mainRobot.drive.trajectoryBuilder(initPose)
                            .strafeLeft(3)
                            .splineToConstantHeading(new Vector2d(108, -13), Math.toRadians(180))
                            .build();

                    toPowerShot = mainRobot.drive.trajectoryBuilder(dropWobble.end())
                            .lineToLinearHeading(new Pose2d(70,14,0))
                            .build();

                    powerShotStrafeOne = mainRobot.drive.trajectoryBuilder(toPowerShot.end())
                            .strafeLeft(7)
                            .build();

                    powerShotStrafeTwo = mainRobot.drive.trajectoryBuilder(powerShotStrafeOne.end())
                            .strafeLeft(7)
                            .build();

                    getSecondWobble = mainRobot.drive.trajectoryBuilder(powerShotStrafeTwo.end())
                            .lineToLinearHeading(new Pose2d(40, 24, Math.toRadians(20)))
                            .build();

                    grabSecondWobble = mainRobot.drive.trajectoryBuilder(getSecondWobble.end())
                            .back(6,
                                    new MinVelocityConstraint(
                                            Arrays.asList(
                                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                    new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                            )
                                    ),
                                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

                    dropSecondWobble = mainRobot.drive.trajectoryBuilder(grabSecondWobble.end())
                            .lineToLinearHeading(new Pose2d(108, 8, Math.toRadians(90)))
                            .build();

                    toLineFromWobble = mainRobot.drive.trajectoryBuilder(dropSecondWobble.end())
                            .strafeLeft(48)
                            .build();

                    break;


            }

        }

    }

}
