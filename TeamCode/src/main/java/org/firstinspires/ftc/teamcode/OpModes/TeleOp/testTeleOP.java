package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Base.AutonomousBaseV1;
import org.firstinspires.ftc.teamcode.Base.RobotRunType;

@TeleOp (name = "Test TeleOp" )
public class testTeleOP extends AutonomousBaseV1 {

    @Override
    public void runOpMode() {

        //set up robot for teleop
        initRobotV2(RobotRunType.AUTONOMOUS);
        double collectorPow;

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
        }

        while (opModeIsActive()) {


            /** //////////////////////////////////
             ///////////////Driver 1///////////////
             ////////////////////////////////// **/

            //control drive train
            FieldCentricDrive();
            //allows reset of gyroscope when aligned with driver preference
            if (gamepad1.a && gamepad1.b) {
                resetAngle();
            }
            /** ////////////////////////////////
             * ///////////Driver 2//////////////
             ///////////////////////////////**/

            /*
            // The Dozer
            if (gamepad2.dpad_up) {
                firstJoint.setPower(1);
            } else
                firstJoint.setPower(0);
            if (gamepad2.y) {
                firstJoint.setPower(-1);
            } else {
                firstJoint.setPower(0);
            }
            // The Tazer
            if (gamepad2.dpad_right) {
                secondJoint.setPower(0.6);
            } else {
                secondJoint.setPower(0);
            }
            if (gamepad2.b) {
                secondJoint.setPower(-0.6);
            } else if (gamepad2.x) {
                secondJoint.setPower(0);
            }

             */
            // Claw

            //The Killdozer
            if (gamepad2.dpad_up) {
                lift.setPower(1);
            } else if (gamepad2.y) {
                lift.setPower(-1);
            } else {
                lift.setPower(0);
            }

            // The Tazer
            if (gamepad2.dpad_right) {
                score.setPower(1);
            } else if (gamepad2.b) {
                score.setPower(-1);
            } else {
                score.setPower(0);
            }



            if (gamepad2.left_bumper) {
                grab();
            }
            if (gamepad2.right_bumper) {
                release();
            }

        }
    }
}
