package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.AutonomousBaseV1;
import org.firstinspires.ftc.teamcode.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.Base.arm;


@Autonomous(name = "Push Samples to Human Player" )
public class Specimen extends AutonomousBaseV1 {

    @Override
    public void runOpMode() {
        arm am = new arm(this);
        //set up robot for teleop
        initRobotV2(RobotRunType.AUTONOMOUS);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
        }
        waitForStart();

        drive(.4,6);
        waitSec(.2);
        strafeRot(.7, 2.5);

        am.scoreToPos(2200, .4);
        waitSec(.3);
        drive(.4, 3);
        waitSec(.3);
        release();
        
        am.scoreToPos(0,.4);
    }

}