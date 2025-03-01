package org.firstinspires.ftc.teamcode.Base;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import  com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public abstract class RobotHardware extends RobotBaseV1 {


    // Declares drive motors
    protected DcMotor leftFront;
    protected DcMotor leftBack;
    protected DcMotor rightFront;
    protected DcMotor rightBack;
    // Declares janky arm motors
    protected DcMotor firstJoint;
    protected DcMotor secondJoint;
    // Declares servos for arm(just for the claw)
    protected Servo claw;
    protected Servo theWrist;
    protected Servo thePinch;
    //Declares Linear Slide motors
    protected DcMotor lift;
    protected DcMotor score;
    protected DcMotor intake;
    public static double armTarget;



    // declares gyro and gyro variables
    protected IMU imu;
    protected Orientation lastAngles = new Orientation();
    protected Orientation angles;
    protected double globalAngle;
    protected int heading;

    //final variables for moving robot to distance
    protected final double WHEEL_DIAMETER = 4;
    protected final double WHEEL_CIRC = WHEEL_DIAMETER * Math.PI;
    protected final double ultraplanetary_PPR = 28;

    protected final double neverest20ppr = 537.6;
    /* this ultraplanetary setting is for the base motor and the 5:1 gearbox
     */
    protected final double DRIVE_GEAR_RATIO = 20;
    protected final int ARM_RATIO = 80;
    protected final double armPPR = 384.5;
    private final double SERVO_GRAB_POSITION = 0;
    private final double SERVO_RELEASE_POSITION = 0.2;

    protected void initRobotV2(RobotRunType robotRunType) {
        // set up drive motors
        leftFront = hardwareMap.dcMotor.get("lf");
        leftBack = hardwareMap.dcMotor.get("lb");
        rightFront = hardwareMap.dcMotor.get("rf");
        rightBack = hardwareMap.dcMotor.get("rb");


        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // set up arm motors
        // set up pincer servo
        claw = hardwareMap.servo.get("claw");
        thePinch = hardwareMap.servo.get("thepinch");
        theWrist = hardwareMap.servo.get("thewrist");

        //set up linear slides
        lift = hardwareMap.dcMotor.get("lift");
        score = hardwareMap.dcMotor.get("score");
        intake = hardwareMap.dcMotor.get("intake");

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        score.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        score.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        score.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        // initialize gyro if starting in autonomous
        if (robotRunType == RobotRunType.AUTONOMOUS) {


//            initialize gyro
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//            parameters.mode = BHI260IMU.;
//            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//            parameters.loggingEnabled = false;

            imu = hardwareMap.get(BHI260IMU.class, "imu");

            imu.initialize(
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                    RevHubOrientationOnRobot.UsbFacingDirection.UP

                            )
                    )
            );

            //post to telemetry when gyro is calibrating
            telemetry.addData("Mode", "Calibrating");
            telemetry.update();

            //post to telemetry when gyro is calibrated


            telemetry.addData("Mode", "waiting for start");
//            telemetry.addData("imu calibration", imu.getCalibrationStatus().toString());
            telemetry.update();


        }


    }

    protected void setDrivePower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }


    protected void stopDrive(){setDrivePower(0, 0, 0, 0);}

    protected void MecanumFormula(double forward, double strafe, double turning) {
        double lfPower, lbPower, rfPower, rbPower;
        double max;

        //reset powers
        lfPower = 0;
        lbPower = 0;
        rfPower = 0;
        rbPower = 0;

        //forward and backward
        lfPower += forward;
        lbPower += forward;
        rfPower += forward;
        rbPower += forward;

        //strafing
        lfPower += strafe;
        lbPower -= strafe;
        rfPower -= strafe;
        rbPower += strafe;

        //turning
        lfPower += turning;
        lbPower += turning;
        rfPower -= turning;
        rbPower -= turning;

        //scale powers of exceeding those of the motor
        max = Math.abs(lfPower);
        if (Math.abs(lbPower) > max) {
            max = Math.abs(lbPower);
        }
        if (Math.abs(rfPower) > max) {
            max = Math.abs(rfPower);
        }
        if (Math.abs(rbPower) > max) {
            max = Math.abs(rbPower);
        }

        if (max > 1) {
            lbPower /= max;
            lfPower /= max;
            rbPower /= max;
            rfPower /= max;
        }

        setDrivePower(rfPower, lfPower, rbPower, lbPower);
    }
        protected void FieldCentricDrive(){
            double x1 = gamepad1.left_stick_x, y1 = -gamepad1.left_stick_y;
            double v = Math.sqrt(x1 * x1 + y1 * y1);
            double theta = Math.atan2(x1, y1);
            double current = Math.toRadians(getGlobal() % 360);
            drive(theta + current, v, gamepad1.right_stick_x);
            // if you pull out field centric just pull out current and it will work
        }

        protected static class Wheels {
            public double lf, lr, rf, rr;

            public Wheels(double lf, double rf, double lr, double rr) {
                this.lf = lf;
                this.rf = rf;
                this.lr = lr;
                this.rr = rr;
            }
        }
        public void setArmTarget(double b) {
            armTarget = b;
    }
        protected Wheels getWheels(double direction, double velocity, double rotationVelocity) {
            final double vd = velocity;
            final double td = direction;
            final double vt = rotationVelocity;

            double s = Math.sin(td + Math.PI / 4.0);
            double c = Math.cos(td + Math.PI / 4.0);
            double m = Math.max(Math.abs(s), Math.abs(c));
            s /= m;
            c /= m;

            final double v1 = vd * s + vt;
            final double v2 = vd * c - vt;
            final double v3 = vd * c + vt;
            final double v4 = vd * s - vt;

            // Ensure that none of the values go over 1.0. If none of the provided values are
            // over 1.0, just scale by 1.0 and keep all values.
            double scale = ma(1.0, v1, v2, v3, v4);

            return new Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
        }

        protected static double ma(double... xs) {
            double ret = 0.0;
            for (double x : xs) {
                ret = Math.max(ret, Math.abs(x));
            }
            return ret;
        }

        protected void drive(double direction, double velocity, double rotationVelocity) {
            Wheels w = getWheels(direction, velocity, rotationVelocity);
            leftFront.setPower(w.lf);
            rightFront.setPower(w.rf);
            leftBack.setPower(w.lr);
            rightBack.setPower(w.rr);
        }

        protected double getGlobal(){
            angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            globalAngle += deltaAngle;

            lastAngles = angles;


            telemetry.addData("global theta: ", globalAngle);
            telemetry.addData("imu theta: ", angles.firstAngle);
            telemetry.update();
            return globalAngle;


        }
        protected void grab() {
        claw.setPosition(SERVO_GRAB_POSITION);

    }
        protected void release() {
        claw.setPosition(SERVO_RELEASE_POSITION);

    }


}


