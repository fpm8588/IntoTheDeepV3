package org.firstinspires.ftc.teamcode.Base;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class arm {
    //Objects
    public DcMotor score;
    Servo claw;


    //VARIABLES

    //Vars for Slide Positions
    public int bottomSlidePos = 0;
    public int highClipSlidePos = 4050; //temp value
    public int lowBucketSlidePos = 2250; //temp value
    public int touchBarSlidePos = 400;
    public final int V_SLIDE_MAX = 4375;
    public int targetPos = bottomSlidePos;

    public final double tprArm = 384.5;
    public double slidePower = 1; //temp value

    //Vars for Bucket Dumping Positions
    public double bucketOutPos = 0.3;
    public double bucketRegPos = 0.65;

    public double targetBucketPos = bucketRegPos;

    //Vars for specimen claw
    public double clawOpen = 1;
    public double clawClose = 0;

    public arm(LinearOpMode opMode) {
        score = opMode.hardwareMap.get(DcMotor.class, "score");
        claw = opMode.hardwareMap.get(Servo.class, "claw");

        score.setDirection(DcMotorSimple.Direction.REVERSE);
        score.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        score.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Vertical Slide to Position
    public void scoreToPos(int pos, double power) {
        score.setTargetPosition(pos);
        score.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        score.setPower(power);
    }

    //Get Vertical Slide Position
    public int getScorePos() {
        return score.getCurrentPosition();
    }

    public void scoreToPow(double power) {
        if (score.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            score.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        score.setPower(power);

    }


}
