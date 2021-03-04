package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RRMainRobot;
import org.firstinspires.ftc.teamcode.Components.Software.Pipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

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
        //mainRobot.shooter.startShooter();

        telemetry.addData("Stack Height:", stackHeight(pipeline.getContourArea()));
        double height = stackHeight(pipeline.getContourArea());
        telemetry.update();

        phoneCam.stopStreaming();

        switch((int) height) {
            case 0:

                mainRobot.drive.setPoseEstimate(initPose);

                Trajectory dropWobble = mainRobot.drive.trajectoryBuilder(initPose)
                        .splineToConstantHeading(new Vector2d(6, -24), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(60, -24), Math.toRadians(180))
                        .build();

                mainRobot.drive.followTrajectory(dropWobble);

                mainRobot.deng(1000);

                mainRobot.wobbleGoalArm.down();
                mainRobot.deng(1000);
                mainRobot.wobbleGoalArm.release();
                mainRobot.deng(500);
                mainRobot.wobbleGoalArm.lift();
                mainRobot.deng(500);

                Trajectory toPowerShot = mainRobot.drive.trajectoryBuilder(dropWobble.end())
                        .splineTo(new Vector2d(60, 24), 0)
                        .build();

                Trajectory powerShotStrafe1 = mainRobot.drive.trajectoryBuilder(toPowerShot.end())
                        .strafeLeft(4)
                        .build();



                break;
            case 1:

                break;
            case 2:

                break;
        }



    }

}
