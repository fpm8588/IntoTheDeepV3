package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.AutonomousBaseV1;
import org.firstinspires.ftc.teamcode.Base.RobotRunType;

@Autonomous(name = "Push Samples to Human Player" )
public class sampleToHumanPlayer extends AutonomousBaseV1 {
    @Override
    public void runOpMode() {

        //set up robot for teleop
        initRobotV2(RobotRunType.AUTONOMOUS);
        double collectorPow;

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
        }
        waitForStart();



        drive(0.4,10);
        waitSec(0.3);
        strafeRot(-0.5,0.6);
        waitSec(0.4);
        drive(0.4, 45);
        waitSec(0.4);
        turnHeading(0.3, -90);
        waitSec(0.5);
        drive(0.4, 12);
        waitSec(.3);
        turnHeading(0.3, -180);
        waitSec(.5);
        drive(0.5, 50);
        waitSec(.5);
        drive(-0.5, 52);
        waitSec(.2);
        strafeRot(.5, .5);
        drive(.5, 50);
        waitSec(.4);
        drive(-.5, 50);
        waitSec(7);
        drive(.4, 55);



    }
}
