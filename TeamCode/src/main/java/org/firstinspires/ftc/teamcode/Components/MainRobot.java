package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Hardware.Drivebase;
import org.firstinspires.ftc.teamcode.Components.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Components.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Components.Hardware.WobbleGoalArm;

public class MainRobot {

    Shooter shooter;
    Intake intake;
    WobbleGoalArm wobbleGoalArm;
    //TODO: Make not public
    public Drivebase drivebase;

    //TODO: Probably make not public
    public OdometryTracker odometryTracker;

    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        wobbleGoalArm = new WobbleGoalArm(hardwareMap, telemetry);
        drivebase = new Drivebase(hardwareMap, telemetry);

        odometryTracker = new OdometryTracker(hardwareMap, telemetry);

    }

}
