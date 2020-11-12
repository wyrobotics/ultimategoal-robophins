package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Components.GlobalPositioning.*;

@TeleOp
public class ResetPosition extends LinearOpMode {

    public void runOpMode() {

        robotX = 0;
        robotY = 0;
        robotTheta = 0;

    }

}
