package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.MainRobot;
import org.firstinspires.ftc.teamcode.Components.Software.Sampler;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class MainAutonLow extends LinearOpMode {

    private MainRobot mainRobot;

    OpenCvInternalCamera phoneCam;

    double contourArea = 0;

    int stackHeight(double area) { return (area < 100) ? 0 : ((area > 450) ? 2 : 1); }

    @Override
    public void runOpMode() throws InterruptedException {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        phoneCam.setPipeline(new MainAutonLow.Pipeline());

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        //phoneCam.setFlashlightEnabled(true);

        mainRobot.wobbleGoalArm.grab();

        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();
        //mainRobot.shooter.startShooter();

            telemetry.addData("Stack Height:", stackHeight(contourArea));
            double height = stackHeight(contourArea);
            telemetry.update();

            phoneCam.stopStreaming();
            //phoneCam.closeCameraDevice();

            mainRobot.drivebase.timedMovement(0.5, 0, 0, 1500, 500);
            mainRobot.deng(500);

            mainRobot.drivebase.timedMovement(0,-0.75,0,2500 + (height * 750),500);
            mainRobot.deng(500);

            if(height == 1) {
                mainRobot.drivebase.timedMovement(-0.5, 0, 0, 1750, 500);
                mainRobot.deng(500);
            }

            mainRobot.wobbleGoalArm.lift();
            mainRobot.deng(1000);
            mainRobot.wobbleGoalArm.release();
            mainRobot.deng(500);
            mainRobot.wobbleGoalArm.down();
            mainRobot.deng(1000);

            if(height != 1) {
                mainRobot.drivebase.timedMovement(-0.5, 0, 0, 1750, 500);
                mainRobot.deng(500);
            }

            if(height == 2) { mainRobot.drivebase.timedMovement(0, 0.5, 0, 2500, 500); }
            if(height == 1) { mainRobot.drivebase.timedMovement(0, 0.5, 0, 1500, 500); }
            mainRobot.deng(500);



        mainRobot.drivebase.turn180();
        mainRobot.deng(1000);


        //THIS VALUE iS THE power fOR THE SHots (the oNE IN simpleshoot that says power)
        mainRobot.shooter.simpleShoot(0.72);
        mainRobot.deng(1000);
        mainRobot.shooter.flick();
        mainRobot.deng(1000);
        mainRobot.shooter.unflick();;
        mainRobot.deng(50);
        mainRobot.jig(2);
        mainRobot.shooter.flick();
        mainRobot.deng(1000);
        mainRobot.shooter.unflick();
        mainRobot.deng(50);
        mainRobot.jig(2);
        mainRobot.shooter.flick();
        mainRobot.deng(1000);
        mainRobot.shooter.unflick();
        mainRobot.deng(1000);
        //mainRobot.deng(8000);

        mainRobot.shooter.simpleShoot(0);

        mainRobot.drivebase.timedMovement(0,0.5,0,1000,500);

        mainRobot.hardwareThreadExecutor.shutdownExecutor();

        mainRobot.drivebase.discOrtho(0,0,0);

    }

    public class Pipeline extends OpenCvPipeline {

        Mat inputRed = new Mat();
        Mat inputGreen = new Mat();
        Mat inputBlue = new Mat();

        private List<MatOfPoint> yellowContours(Mat input) {

            Mat yellowMask = new Mat(input.size(), 0);



            Imgproc.GaussianBlur(input,input,new Size(3,3),0);

            List<Mat> channels = new ArrayList<>();

            Core.split(input,channels);

            Imgproc.threshold(channels.get(0),inputRed,180,200,Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(1),inputGreen,20,175,Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(2),inputBlue,100,255,Imgproc.THRESH_BINARY_INV);

            Core.bitwise_and(inputRed,inputGreen,yellowMask);
            Core.bitwise_and(yellowMask,inputBlue,yellowMask);

            List<MatOfPoint> yellowContours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(yellowMask, yellowContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            return yellowContours;

        }

        @Override
        public Mat processFrame(Mat input) {

            List<MatOfPoint> yellowContours = yellowContours(input);

            double newArea;
            double maxArea = 0;

            Rect biggestRect = new Rect(0,0, 1,1);

            for (int i = 0; i < yellowContours.size(); i++) {

                newArea = Imgproc.contourArea(yellowContours.get(i));

                if (newArea > maxArea) {

                    maxArea = newArea;

                    biggestRect = Imgproc.boundingRect(yellowContours.get(i));

                }
            }

            Imgproc.rectangle(input, biggestRect, new Scalar(200, 0, 0), 2);

            contourArea = maxArea;

            return input;

        }

    }

}
