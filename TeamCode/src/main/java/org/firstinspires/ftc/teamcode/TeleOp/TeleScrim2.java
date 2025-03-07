package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCenterStage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="TeleScrim2", group="Iterative Opmode")

public class TeleScrim2 extends OpMode {
    RobotCenterStage robot = new RobotCenterStage();
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    double initialST; // Initial time for storage button timer
    double slowTime; // initial time for slowmode timeout
    double slowTime2; // initial time for super slowmode timeout
    double liftTimestart, liftTime;  // initial time for lifter timing

    double checkTimeL, checkTimeH;

    double bfrReset;

    // set up variables for motor powers
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double strafingConstant = 1.5;
    double lifterPower;
    int smalllift=500;//400; //400 with faster motor

    enum wristStates{
        Off, On
    }

    enum elbowStates{
        Off, On
    }

    enum lifterStates {
        Home, Low, Middle, High, Manual, Junction, Between
    }

    // lifter encoder positions
    // b is 0
    // a is -1150
    // x is -1700
    // y is -2600

    // state machine variables

    lifterStates lifterLocation = lifterStates.Home;

    wristStates wristStatus = wristStates.Off;
    elbowStates elbowStatus = elbowStates.Off;

    // Setup booleans for state machines
    boolean firstHomeLift = true;
    boolean movingLifter = false;


    //lifting motor new PIDF values
    public  double NEW_P = 10;//13; //15
    public double NEW_I = 3;//3; //3
    public  double NEW_D =0.2;// 1.5; //1.5
    public  double NEW_F =12.56;// 14;  //was 12.6

    @Override
    public void init() {
        robot.initTele(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        robot.resetDriveEncoders();
        // run using encoders makes it slower but drive very straight
        robot.startDriveEncoders();

        robot.lifter.setTargetPositionTolerance(15);//was 15 at kent
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        // PIDFCoefficients pidNew = new PIDFCoefficients(10.0, 3.0 0, 10);
        robot.lifter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);

        robot.elbow.setPosition(0.5);
        robot.wrist.setPosition(0);
        robot.drone.setPosition(0);

        robot.lifter.setTargetPositionTolerance(15);//was 15 at kent
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double slow = 1.0;

        Pose2d poseEstimate = drive.getPoseEstimate();

        if(gamepad2.right_trigger != 0) {
            slow = 0.35;
        }
        else {
            slow = 1.0;
        }
        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y*slow,
                -gamepad1.left_stick_x*slow
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();

        lifterPower = gamepad2.left_stick_y;

        // Make sure driving power is -1 to 1 and set max/min values
        frontLeftPower = Range.clip(frontLeftPower, -1, 1);
        frontRightPower = Range.clip(frontRightPower, -1, 1);
        backLeftPower = Range.clip(backLeftPower, -1, 1);
        backRightPower = Range.clip(backRightPower, -1, 1);

        lifterPower = Range.clip(lifterPower, -1, 1);

        //******************************************************************************************

        // Lifter implementation
//
//        if (getRuntime() > checkTimeL && getRuntime() < checkTimeH){
//            if(robot.lifter.isBusy()) {
//                robot.lifter.setPower(0);
//            }
//        }
//
//        //override for sahas driver 1
//        if (gamepad1.right_trigger > 0.5 && movingLifter){      // lifter override
//            robot.lifter.setPower(0);
//            movingLifter = false;
//        }
        robot.lifter.setPower(lifterPower);

        robot.intake.setPower(-gamepad2.right_stick_y);


        //button debounce
        if(elbowStatus == elbowStates.Off && gamepad2.a){
            elbowStatus = elbowStates.On;
            robot.elbow.setPosition(0.5);
        }
        else if(elbowStatus == elbowStates.On && gamepad2.a){
            elbowStatus = elbowStates.Off;
            robot.elbow.setPosition(1);
        }

        if(wristStatus == wristStates.Off && gamepad2.y){
            wristStatus = wristStates.On;
            robot.wrist.setPosition(0);
        }
        else if(wristStatus == wristStates.On && gamepad2.y){
            wristStatus = wristStates.Off;
            robot.wrist.setPosition(1);
        }

        if(gamepad2.x){
            robot.lifter.setTargetPosition(-50);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(0.6);
            robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // robot.lifter.setPower(lifterPower);
        telemetry.addData("Lifter time ", liftTime);
        telemetry.addData("Lifter Power: ", lifterPower);
        telemetry.addData("Lifter ticks before reset", bfrReset);
        telemetry.addData("current Lifter Ticks: ", robot.lifter.getCurrentPosition());
        // update telemetry of drive motors in order to figure out why the robot is not driving straight 11/8/22
        telemetry.addData("front left ticks", robot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("back left ticks", robot.backLeftMotor.getCurrentPosition());
        telemetry.addData("front right ticks", robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("back right ticks", robot.backRightMotor.getCurrentPosition());
        telemetry.addData("intake power", drive.intake.getPower());
        telemetry.addData("intake encoder", drive.intake.getCurrentPosition());

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("FL Encoder", drive.leftFront.getCurrentPosition());
        telemetry.addData("RL Encoder", drive.leftRear.getCurrentPosition());
        telemetry.addData("FR Encoder", drive.leftFront.getCurrentPosition());
        telemetry.addData("RR Encoder", drive.leftRear.getCurrentPosition());
        telemetry.addData("FL Power", drive.leftFront.getPower());
        telemetry.addData("RL Power", drive.leftRear.getPower());
        telemetry.addData("FR Power", drive.leftFront.getPower());
        telemetry.addData("RR Power", drive.leftRear.getPower());

        telemetry.update();
        //telemetry.addData("right encoder", robot.rightEncoder.getCurrentPosition());
        //telemetry.addData("left encoder", robot.leftEncoder.getCurrentPosition());
        //telemetry.addData("lateral encoder", robot.frontEncoder.getCurrentPosition());

        telemetry.addData("At home? ", lifterLocation == lifterStates.Home);

        // Update telemetry at end
        telemetry.update();

    }

    public void stop() {
        robot.stopAllMotors();
    }

}
