package org.firstinspires.ftc.teamcode.OpModes.Test;

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
public class ShooterTest extends LinearOpMode {

    private MainRobot mainRobot;

    OpenCvInternalCamera phoneCam;

    double contourArea = 0;

    int stackHeight(double area) { return (area < 100) ? 0 : ((area > 450) ? 2 : 1); }

    @Override
    public void runOpMode() throws InterruptedException {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        mainRobot.wobbleGoalArm.grab();

        waitForStart();

        mainRobot.intake.intake();
        mainRobot.shooter.simpleShoot(0.87);

        mainRobot.deng(8000);

        mainRobot.intake.intake(0);
        mainRobot.shooter.simpleShoot(0);

        mainRobot.drivebase.discOrtho(0,0,0);

    }

}
