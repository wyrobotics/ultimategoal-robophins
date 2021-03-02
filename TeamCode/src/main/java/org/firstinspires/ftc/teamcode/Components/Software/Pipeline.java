package org.firstinspires.ftc.teamcode.Components.Software;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Pipeline extends OpenCvPipeline {

    Mat inputRed = new Mat();
    Mat inputGreen = new Mat();
    Mat inputBlue = new Mat();

    double contourArea;

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

    public double getContourArea() { return contourArea; }

}