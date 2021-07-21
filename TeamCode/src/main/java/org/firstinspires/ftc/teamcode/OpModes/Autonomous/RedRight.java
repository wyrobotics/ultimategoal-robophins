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
import org.firstinspires.ftc.teamcode.Components.Software.GlobalPositioning;
import org.firstinspires.ftc.teamcode.Components.Software.Pipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Arrays;
import java.util.Vector;

@Autonomous
public class RedRight extends LinearOpMode {

    RRMainRobot mainRobot;

    OpenCvInternalCamera phoneCam;

    Pipeline pipeline = new Pipeline();

    int stackHeight(double area) { return (area < 100) ? 0 : ((area > 450) ? 2 : 1); }

    @Override
    public void runOpMode() {

        //Init stuff

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

        mainRobot.wacker.upperWacker();


        Pose2d initPose = new Pose2d(0, -2, Math.toRadians(180));


        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();

        telemetry.addData("Stack Height:", stackHeight(pipeline.getContourArea()));
        int height = stackHeight(pipeline.getContourArea());
        telemetry.update();

        phoneCam.stopStreaming();

        AutonTrajectories autonTrajectories = new AutonTrajectories(height);

        mainRobot.drive.setPoseEstimate(autonTrajectories.initPose);

        //Start actual auton

        mainRobot.drive.followTrajectory(autonTrajectories.dropWobble);

        //mainRobot.deng(200);

        mainRobot.wobbleGoalArm.lift();
        mainRobot.deng(700);
        mainRobot.wobbleGoalArm.release();
        mainRobot.deng(700);
        mainRobot.wobbleGoalArm.down();
        mainRobot.deng(500);

        mainRobot.drive.followTrajectory(autonTrajectories.prepToShoot);

        mainRobot.deng(200);

        mainRobot.shooter.setpoint = 820;

        mainRobot.deng(2000);
        mainRobot.shooter.flick();

        //mainRobot.shooter.setpoint = 790;

        mainRobot.deng(1000);
        mainRobot.shooter.unflick();
        mainRobot.deng(1000);

        mainRobot.wacker.lowerWacker();
        mainRobot.deng(500);
        mainRobot.wacker.upperWacker();

        mainRobot.deng(1000);
        mainRobot.shooter.flick();
        mainRobot.deng(1000);
        mainRobot.shooter.unflick();
        mainRobot.deng(1000);

        mainRobot.wacker.lowerWacker();
        mainRobot.deng(500);
        mainRobot.wacker.upperWacker();

        mainRobot.deng(1000);
        mainRobot.shooter.flick();
        mainRobot.deng(1000);
        mainRobot.shooter.unflick();
        mainRobot.deng(1000);


        GlobalPositioning.poseMemory = mainRobot.drive.getPoseEstimate();

        mainRobot.shooter.setpoint = 0;
        mainRobot.shooter.disableController();
        mainRobot.shooter.shutdownShooter();
        mainRobot.hardwareThreadExecutor.shutdownExecutor();
        mainRobot.shooter.simpleShoot(0);

        mainRobot.drive.followTrajectory(autonTrajectories.toLine);

    }

    private class AutonTrajectories {

        Trajectory dropWobble;

        Trajectory prepToShoot;

        Trajectory jig1;

        Trajectory jig2;

        Trajectory toLine;

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(180));

        public AutonTrajectories(int stackHeight) {

            switch(stackHeight) {

                case 0:

                    dropWobble = mainRobot.drive.trajectoryBuilder(initPose)
                            .back(60)
                            .build();

                    prepToShoot = mainRobot.drive.trajectoryBuilder(dropWobble.end())
                            .lineToLinearHeading(new Pose2d(60,15,0))
                            .build();

                    jig1 = mainRobot.drive.trajectoryBuilder(prepToShoot.end())
                            .forward(8)
                            .build();

                    jig2 = mainRobot.drive.trajectoryBuilder(jig1.end())
                            .back(8, new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                        )
                ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

                    toLine = mainRobot.drive.trajectoryBuilder(prepToShoot.end())
                            .forward(12)
                            .build();

                    break;

                //1 RING STACK
                case 1:

                    dropWobble = mainRobot.drive.trajectoryBuilder(initPose)
                            .back(60)
                            .splineToConstantHeading(new Vector2d(84,10), Math.toRadians(180))
                            .build();

                    prepToShoot = mainRobot.drive.trajectoryBuilder(dropWobble.end())
                            .lineToLinearHeading(new Pose2d(60,15,0))
                            .build();

                    jig1 = mainRobot.drive.trajectoryBuilder(prepToShoot.end())
                            .forward(8)
                            .build();

                    jig2 = mainRobot.drive.trajectoryBuilder(jig1.end())
                            .back(8)
                            .build();

                    toLine = mainRobot.drive.trajectoryBuilder(prepToShoot.end())
                            .forward(12)
                            .build();

                    break;

                //4 RING STACK
                case 2:

                    dropWobble = mainRobot.drive.trajectoryBuilder(initPose)
                            .back(98)
                            .build();

                    prepToShoot = mainRobot.drive.trajectoryBuilder(dropWobble.end())
                            .lineToLinearHeading(new Pose2d(60,15,0))
                            .build();

                    jig1 = mainRobot.drive.trajectoryBuilder(prepToShoot.end())
                            .forward(8)
                            .build();

                    jig2 = mainRobot.drive.trajectoryBuilder(jig1.end())
                            .back(8)
                            .build();

                    toLine = mainRobot.drive.trajectoryBuilder(prepToShoot.end())
                            .forward(12)
                            .build();

                    break;


            }

        }

    }

}
