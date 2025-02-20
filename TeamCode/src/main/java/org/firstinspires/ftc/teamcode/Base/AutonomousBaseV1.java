package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.List;


public abstract class AutonomousBaseV1 extends RobotHardware {
    //resets drive motor encoders


    protected void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive() && rightFront.getCurrentPosition() > 3 && leftFront.getCurrentPosition() > 3) {
        }
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && rightFront.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER && leftFront.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        }
    }

    //resets encoders in arm
    protected void resetArmEncoders() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        score.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive() && firstJoint.getCurrentPosition() > 3 && secondJoint.getCurrentPosition() > 3)

            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        score.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && firstJoint.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER && secondJoint.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            ;
    }

    /**
     * pauses code for a set amount of seconds
     *
     * @param seconds time to wait before resuming code
     */
    protected void waitSec(double seconds) {
        ElapsedTime t = new ElapsedTime(System.nanoTime());
        while (opModeIsActive() && t.time() <= seconds) {
        }
    }

    /**
     * gets the absolute value of the right drive motor encoder positions
     *
     * @return int - absolute value of right front wheel
     */
    protected int getRightAbs() {
        return Math.abs(rightFront.getCurrentPosition());
    }

    /// Same thing as before, just for the left drive
    protected int getLeftAbs() {
        return Math.abs(leftFront.getCurrentPosition());
    }

    /**
     *
     *
     * Movement
     *
     */

    /**
     * sets all drive motors to the same value
     *  power in which the drive motors will be set to
     */
    protected void drive(double power) {
        setDrivePower(power, power, power, power);
    }
    /**
     * drives the robot in a straight line for a given distance
     *
     * @param power speed which the robot should move at
     * @param inches target for which the robot should try to stop at
     */
    protected void drive(double power, double inches) {
        resetEncoders();
        drive(power);
        double targetPosition = inches * ultraplanetary_PPR * DRIVE_GEAR_RATIO / WHEEL_CIRC;
        double starting = getAngle();
        double right = power;
        double left = power;
        while (opModeIsActive() && getLeftAbs() <= targetPosition && getRightAbs() <= targetPosition) {
            double delta = starting - getAngle();
            right = power + (delta / 40);
            left = power - (delta / 40);

            if (Math.abs(right) > 1 || Math.abs(left) > 1) {
                right /= Math.max(right, left);
                left /= Math.max(right, left);
            }
            setDrivePower(left, left, right, right);
        }
        stopDrive();
        telemetry.addLine("Drove " + inches + " inches to target");
        telemetry.update();
    }

    /**
     * moves the robot straight laterally at a given power and distance
     *
     * @param power     speed at which it should move
     * @param rotations number of rotations the wheels should spin
     */
    protected void strafeRot(double power, double rotations) {
        resetEncoders();
        waitSec(0.2);
        setDrivePower(-power, power, power, -power);
        double targetPosition = rotations * ultraplanetary_PPR * DRIVE_GEAR_RATIO;

        while (Math.abs((double) rightFront.getCurrentPosition()) <= targetPosition) {

        }
        stopDrive();
        telemetry.addLine("Strafed " + rotations + " Rotations to target");
        telemetry.update();
    }

    protected void strafeTime(double power, double rotations) {
        resetEncoders();
        setDrivePower(-power, power, power, -power);
        double targetPosition = rotations * ultraplanetary_PPR * DRIVE_GEAR_RATIO;

        waitSec(rotations);
        stopDrive();
        telemetry.addLine("Strafed " + rotations + " Rotations to target");
        telemetry.update();
    }

    protected void driveVector(double target, double direction, double power, double rot, int heading, boolean time) {
        resetEncoders();
        direction = Math.toRadians(direction);
        double adjustment;
        if (rot == 0) {
            adjustment = power / 2;
        } else {
            adjustment = power;
        }
        if (power < 0) {
            adjustment = -adjustment;
        }
        target = target * ultraplanetary_PPR * DRIVE_GEAR_RATIO / WHEEL_CIRC;
        drive(direction, power, 0);
        while (getRightAbs() < target && getLeftAbs() < target && opModeIsActive()) {
            if (Math.abs(getAngle() - heading) > 3)
                if (getAngle() < heading) {
                    drive(direction, power, -adjustment);
                } else if (getAngle() > heading) {
                    drive(direction, power, adjustment);
                } else {
                    drive(direction, power, 0);
                }
        }
        stopDrive();
    }

    /**
     * turns the robot at a given speed to target angle
     *
     * @param power        speed at which to turn
     * @param degreeTarget target angle which it will try to turn to
     */
    protected void turnHeading(double power, double degreeTarget) {
        double heading = getGlobal();

        //turn until gyro value is met
        while (opModeIsActive() && Math.abs(heading - degreeTarget) > 0.3) {
            if (heading > degreeTarget) {
                setDrivePower(power, power, -power, -power);
            }
            if (heading < degreeTarget) {
                setDrivePower(-power, -power, power, power);
            }
            heading = getGlobal();
        }
        stopDrive();
        telemetry.addLine("Turned " + degreeTarget + " degrees to target");
        telemetry.update();
    }
    ////////////////////////////////////////
    ///////////////arm/////////////////////
    //////////////////////////////////////
    /**
     * moves the arm a set amount of degrees at a given speed
     * @param degrees amount to move the arm
     * @param power speed/direction to move the arm
     */
    protected void extendArm(int degrees, double power){

        int target = (int) (ultraplanetary_PPR * ARM_RATIO * degrees / 360);
        firstJoint.setPower(power);
        while (Math.abs(firstJoint.getCurrentPosition()) < target && opModeIsActive()){
            firstJoint.setPower(power);
            secondJoint.setPower(power);
        }
        firstJoint.setPower(0);
        secondJoint.setPower(0);

        telemetry.addLine("Rotated Arm " + degrees + " degrees");
        telemetry.update();
    }
    ///////////////////////////////////
    ///////////////Gyro////////////////
    ///////////////////////////////////

    /**
     * resets the angle of the gyroscope method
     */
    protected void resetAngle() {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * gets the angle of the gyro
     * @return int - angular heading of the robot
     */
    private int getAngle() {

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int) angles.firstAngle;
    }


}
