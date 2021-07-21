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
public class DoALilParkin extends LinearOpMode {

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

        mainRobot.drive.followTrajectory(autonTrajectories.goBack);

    }

    private class AutonTrajectories {

        Trajectory goBack;

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(180));

        public AutonTrajectories(int stackHeight) {

            goBack = mainRobot.drive.trajectoryBuilder(initPose)
                    .back(70)
                    .build();


        }

    }

}
