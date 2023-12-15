package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


// object recognition imports
//import org.firstinspires.ftc.teamcode.pipelines.ColorVals;
//import org.firstinspires.ftc.teamcode.pipelines.CenterStagePipeline;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public class RobotCenterStage {

    // Define Motors, Sensors, and Variables *******************************************************
    public DcMotor frontLeftMotor = null;

    public OpenCvCamera getCamera() {
        return camera;
    }

    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    public DcMotor intake = null;

    public Servo wrist = null;
    public Servo elbow = null;
    public Servo drone = null;

    public DcMotorEx lifter = null;

    public DigitalChannel lifterSwitch1 = null;
    public DigitalChannel lifterSwitch2 = null;

    public Encoder leftEncoder = null;
    public Encoder rightEncoder = null;
    public Encoder frontEncoder = null;

    public OpMode systemTools;

    public WebcamName cam = null;
    //public WebcamName webacma1 = null;
    // public WebcamName webacma2 = null;


    HardwareMap hwMap;
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;

    //***********************************************************************************************
    // cone recognition variables
    OpenCvCamera camera;

    FtcDashboard dashboard;
//    CenterStage pipeline;

    //Init Methods *********************************************************************************
    public void initAuto(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;

        setupMotorsGeneric();

        startDriveEncoders();

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        //lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter.setTargetPositionTolerance(15);

        stopAllMotors();
        resetDriveEncoders();   // added on 2-13-22
        startDriveEncoders();
    }

    public void initAutoTester(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;
        lifter.setTargetPositionTolerance(15);
    }

    public void initAutoRR(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;

        setUpMotorsRR();

        lifter.setTargetPositionTolerance(15);

    }

    //Init Methods**********************************************************************************
    public void initTele(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;

        setupMotorsGeneric();

        startDriveEncoderless();

        // reset lifter encoder
        //lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


     /*   PIDFCoefficients pidNew = new PIDFCoefficients(10.0, 3.0 0, 10);
        lifter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);*/

        //carousel.setMode(DcMotorex.RunMode.RUN_USING_ENCODER);



        //frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopAllMotors();
        startDriveEncoderless();
        lifter.setTargetPositionTolerance(15);
    }

    public void setupMotorsGeneric() {
        // Define and Initialize Motors
        // MOTOR DISABLED FOR LIFTER TESTING
        frontLeftMotor  = hwMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hwMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hwMap.get(DcMotor.class, "leftRear");
        backRightMotor = hwMap.get(DcMotor.class, "rightRear");
        intake = hwMap.get(DcMotor.class, "intake");
        lifter = hwMap.get(DcMotorEx.class, "lifter");
        wrist = hwMap.get(Servo.class,"wristServo");
        elbow = hwMap.get(Servo.class,"elbowServo");
        drone = hwMap.get(Servo.class, "droneServo");
        DcMotor[] driveMotors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
        //lifter = hwMap.get(DcMotorEx.class, "lifter");
        //lifterSwitch1 = hwMap.get(DigitalChannel.class, "lifter_switch_one");
        //lifterSwitch2 = hwMap.get(DigitalChannel.class, "lifter_switch_two");
        //leftEncoder = new Encoder(hwMap.get(DcMotorEx.class, "od_lf"));
        //rightEncoder = new Encoder(hwMap.get(DcMotorEx.class, "od_rt"));
        //frontEncoder = new Encoder(hwMap.get(DcMotorEx.class, "od_lat"));
    }

    public void setUpMotorsRR() {
        // Define and Initialize Motors
        // MOTOR DISABLED FOR LIFTER TESTING
        //frontLeftMotor  = hwMap.get(DcMotor.class, "leftFront");
        //frontRightMotor = hwMap.get(DcMotor.class, "rightFront");
        //backLeftMotor = hwMap.get(DcMotor.class, "leftRear");
        //backRightMotor = hwMap.get(DcMotor.class, "rightRear");
        lifter = hwMap.get(DcMotorEx.class, "lifter");
        wrist = hwMap.get(Servo.class,"wrist_servo");
        elbow = hwMap.get(Servo.class,"elbow_servo");
        drone = hwMap.get(Servo.class,"droneServo");

        DcMotor[] driveMotors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        lifter = hwMap.get(DcMotorEx.class, "lifter");

//        lifterSwitch1 = hwMap.get(DigitalChannel.class, "lifter_switch_one");
//        lifterSwitch2 = hwMap.get(DigitalChannel.class, "lifter_switch_two");
    }


    //Autonomous Movement Commands (forward, turn, strafe, lifter, ect.)

    //Auto go distance function (CM, power, handoff, opmode) - uses gotoTarget
    public void GoDistance(double centimeters, double power, boolean Handoff, LinearOpMode linearOpMode) {
        // holds the conversion factor for TICKS to centimeters
        final double conversionFactor = 17.59; // Number came from testing, may need to be improved

        // FIX REVERSE GODISTANCE MOVEMENT
        //power *= -1;

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversionFactor);

        resetDriveEncoders();

        //Calculate the target for each specific motor
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() + TICKS;

        setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);

        startDriveEncodersTarget();

        setDrivePower(power, power, power, power);

        // keep looping while we are still active, and there is time left, and all motors are running.
        while (linearOpMode.opModeIsActive() &&
                //     (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())) {
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
        }

        if (!Handoff) stopDriveMotors();
    }

    // Rotate Robot (degress,power,op,telOn) - uses position checks
    public void Rotate(int degrees, double power, LinearOpMode linearOpMode, boolean TelemetryOn) {

        final double conversionFactor = 9.6; // for outreach robot: 8.46, for FreightFrenzy robot: 9.6

        if (degrees < 0 && power > 0) power = -power;

        int ticks = (int) abs(Math.round(degrees * conversionFactor));

        if (TelemetryOn) {
            systemTools.telemetry.addData("Status", "Resetting Encoders");
            systemTools.telemetry.update();
        }

        resetDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(),
                    backLeftMotor.getCurrentPosition(),
                    backRightMotor.getCurrentPosition());
            systemTools.telemetry.update();
        }
        //Conversions to rotate
        int FLtarget = frontLeftMotor.getCurrentPosition() + ticks;
        int FRtarget = frontRightMotor.getCurrentPosition() - ticks;
        int BLtarget = backLeftMotor.getCurrentPosition() + ticks;
        int BRtarget = backRightMotor.getCurrentPosition() - ticks;

        startDriveEncoders();
        //Starts to rotate
        // setDrivePower(power, -power, -power, power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        backLeftMotor.setPower(power);

        while (linearOpMode.opModeIsActive() &&
                (abs(frontLeftMotor.getCurrentPosition()) < ticks && abs(frontRightMotor.getCurrentPosition()) < ticks && abs(backLeftMotor.getCurrentPosition()) < ticks && abs(backRightMotor.getCurrentPosition()) < ticks)) {
        }

        stopDriveMotors();

        startDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path", "Complete");
            systemTools.telemetry.addData("counts", ticks);
            systemTools.telemetry.update();
        }

    }

    //*************************************************************************************************
    // Strafe program inputs (cm,power,opmode) - uses position checks POSITIVE DISTACE GOES RIGHT
    public void Strafe(double Centimeters, double Power, LinearOpMode linearOpMode, boolean TelemetryOn) {
        final double conversionFactor = 18.38; // outreach robot: 8.46 FreightFrenzy robot: TBD
        Centimeters = Centimeters * -1;

        if (Centimeters < 0 && Power > 0) Power = -Power;

        int ticks = (int) abs(Math.round(Centimeters * conversionFactor));

        if (TelemetryOn) {
            systemTools.telemetry.addData("Status", "Resetting Encoders");
            systemTools.telemetry.update();
        }

        resetDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(),
                    backLeftMotor.getCurrentPosition(),
                    backRightMotor.getCurrentPosition());
            systemTools.telemetry.update();
        }

        int FLtarget = frontLeftMotor.getCurrentPosition() - ticks;
        int FRtarget = frontRightMotor.getCurrentPosition() + ticks;
        int BLtarget = backLeftMotor.getCurrentPosition() + ticks;
        int BRtarget = backRightMotor.getCurrentPosition() - ticks;

        startDriveEncoders();

        //setDrivePower(Power, -Power, Power, -Power);
        frontLeftMotor.setPower(-Power);
        frontRightMotor.setPower(Power);
        backRightMotor.setPower(-Power);
        backLeftMotor.setPower(+Power);

        while (linearOpMode.opModeIsActive() &&
                (abs(frontLeftMotor.getCurrentPosition()) < ticks || abs(frontRightMotor.getCurrentPosition()) < ticks || abs(backLeftMotor.getCurrentPosition()) < ticks || abs(backRightMotor.getCurrentPosition()) < ticks)) {
        }

        stopDriveMotors();

        startDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path", "Complete");
            systemTools.telemetry.addData("counts", ticks);
            systemTools.telemetry.update();
        }

    }

    //Acceleration movement functions (drive and strafe)

    public void goDistanceAcceleration(int centimeters, double power, boolean handoff, double frontRamp, double backRamp, LinearOpMode linearOpMode) {

        // IMPORTANT: for backramp, subtract the percent from 100. For example, if you want the robot to ramp down for the last 30.0 percent, set it to 70.0

        // Holds the conversion factor for TICKS to centimeters
        final double conversion_factor = 17.59;  // was 22 last season, now 17.59 based on godistance, may need tweaked
        double setPower = 0.0;
        double percent;
        double percent2;
        boolean backwards;

        // Sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }
        backwards = power < 0;
        power = abs(power);

        // Calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        // Debug: Send telemetry message with calculated TICKS;
        systemTools.telemetry.addData("Calculated Counts =", TICKS);

        resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();


        // sets the target position for each of the motor encoders
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() + TICKS;

        //setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);
        frontLeftMotor.setTargetPosition(FLtarget);
        frontRightMotor.setTargetPosition(FRtarget);
        backLeftMotor.setTargetPosition(BLtarget);
        backRightMotor.setTargetPosition(BRtarget);

        // Sets the motors to start running to the position they have been given
        //startDriveEncodersTarget();
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // keep looping while we are still active, and there is time left, and all motors are running.
        while (linearOpMode.opModeIsActive() &&
                //   (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            // Finds out how far into the motion we are (0-100)
            double fLpercent = (double) (frontLeftMotor.getCurrentPosition()) / frontLeftMotor.getTargetPosition() * 100;


            if (fLpercent <= frontRamp) { // front ramp was 30.0
                percent = fLpercent / frontRamp; // Finds out how far into the front ramp (0-1)
                setPower = percent * power; // accelerates from 0-max power

                // Set minimum power to .1 to get the robot started moving
                if (setPower < 0.1) {
                    setPower = 0.1;
                }
            }
            if (fLpercent > frontRamp && fLpercent < backRamp) {
                setPower = power; // power stays at max in the middle of the course
            }
            if (fLpercent >= backRamp) { // back ramp was 70.0
                // Finds out how far into the back ramp(0-backRamp)
                percent2 = fLpercent - backRamp;
                // converts that to a percent on the backramp left (0-1)
                percent = percent2 / (100.0 - backRamp);
                setPower = (1 - percent) * power; // power decreases to zero at the end
                // if (setPower  <0.03) {setPower = 0.03;} // Minimum if needed to stop without rolling down too much disabled for tourney 2 and 3.
            }

            // set the power the motors need to be going at
            if (!backwards) {
                //setDrivePower(setPower, setPower, setPower, setPower);
                frontLeftMotor.setPower(setPower);
                frontRightMotor.setPower(setPower);
                backRightMotor.setPower(setPower);
                backLeftMotor.setPower(setPower);
            } else {
                //setDrivePower(-setPower, -setPower, -setPower, -setPower);
                frontLeftMotor.setPower(-setPower);
                frontRightMotor.setPower(-setPower);
                backRightMotor.setPower(-setPower);
                backLeftMotor.setPower(-setPower);
            }
        }

        if (!handoff) stopDriveMotors();

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        //systemTools.telemetry.update();
    }

    public void strafeAcceleration(int centimeters, double power, boolean handoff, double frontRamp, double backRamp, LinearOpMode linearOpMode) {

        // IMPORTANT: for backramp, subtract the percent from 100. For example, if you want the robot to ramp down for the last 30.0 percent, set it to 70.0

        // Holds the conversion factor for TICKS to centimeters
        final double conversion_factor = 18.38; // Changed to current strafe conversion - modify to match for acceleration
        double setPower = 0.0;
        double percent;
        double percent2;
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;
        boolean left = centimeters < 0;

        centimeters = abs(centimeters);
        power = abs(power);

        // Calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        // Debug: Send telemetry message with calculated TICKS;
        systemTools.telemetry.addData("Calculated Counts =", TICKS);
        //   systemTools.telemetry.update();

        resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        // Sets the target position for each of the motor encoders
        if (left) {
            FLtarget = frontLeftMotor.getCurrentPosition() - TICKS;
            FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
            BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
            BRtarget = backRightMotor.getCurrentPosition() - TICKS;
        } else {
            FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
            FRtarget = frontRightMotor.getCurrentPosition() - TICKS;
            BLtarget = backLeftMotor.getCurrentPosition() - TICKS;
            BRtarget = backRightMotor.getCurrentPosition() + TICKS;
        }

        //setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);
        frontLeftMotor.setTargetPosition(FLtarget);
        frontRightMotor.setTargetPosition(FRtarget);
        backLeftMotor.setTargetPosition(BLtarget);
        backRightMotor.setTargetPosition(BRtarget);

        // Sets the motors to start running to the position they have been given
        //startDriveEncodersTarget();
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // keep looping while we are still active, and there is time left, and all motors are running.
        while (linearOpMode.opModeIsActive() &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            // Finds out how far into the motion we are (0-100)
            double fLpercent = (double) (frontLeftMotor.getCurrentPosition()) / frontLeftMotor.getTargetPosition() * 100;


            if (fLpercent <= frontRamp) { // front ramp was 30.0
                percent = fLpercent / frontRamp; // Finds out how far into the front ramp (0-1)
                setPower = percent * power; // accelerates from 0-max power

                // Set minimum power to .1 to get the robot started moving
                if (setPower < 0.1) {
                    setPower = 0.1;
                }
            }
            if (fLpercent > frontRamp && fLpercent < backRamp) {
                setPower = power; // power stays at max in the middle of the course
            }
            if (fLpercent >= backRamp) { // back ramp was 70.0
                // Finds out how far into the back ramp(0-backRamp)
                percent2 = fLpercent - backRamp;
                // converts that to a percent on the backramp left (0-1)
                percent = percent2 / (100.0 - backRamp);
                setPower = (1 - percent) * power; // power decreases to zero at the end
                //set minimum power to 0.05 to allow the robot to actually hit the target
                if (setPower < 0.05) {
                    setPower = 0.05;
                }
            }

            // set the power the motors need to be going at
            if (left) {
                //setDrivePower(-setPower, setPower, -setPower, setPower);
                frontLeftMotor.setPower(-setPower);
                frontRightMotor.setPower(setPower);
                backRightMotor.setPower(-setPower);
                backLeftMotor.setPower(setPower);
            } else {
                //setDrivePower(setPower, -setPower, setPower, -setPower);
                frontLeftMotor.setPower(setPower);
                frontRightMotor.setPower(-setPower);
                backRightMotor.setPower(setPower);
                backLeftMotor.setPower(-setPower);
            }
        }

        if (!handoff) stopDriveMotors();

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        //systemTools.telemetry.update();
    }
    // Basic movement commands (powers, drivemodes)

    public void setDrivePower(double frontLeftPower, double frontRightPower, double backRightPower, double backLeftPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        backLeftMotor.setPower(backLeftPower);
    }

    // Basic move and target distance
    public void setDriveTarget(int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget) {
        frontLeftMotor.setTargetPosition(frontLeftTarget);
        frontRightMotor.setTargetPosition(frontRightTarget);
        backLeftMotor.setTargetPosition(backLeftTarget);
        backRightMotor.setTargetPosition(backRightTarget);
    }

    public void stopDriveMotors() {
        setDrivePower(0, 0, 0, 0);
    }

    public void wait(long timeout, LinearOpMode linearOpMode) {
        linearOpMode.sleep(timeout);
    }

    //End all movement
    public void stopAllMotors() {
        stopDriveMotors();
        // Add any other motors we use here and set them to 0 power
    }

    // DriveEncoders are reset
    public void resetDriveEncoders() {

//        for (DcMotor motor : driveMotors) {
//             motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void startDriveEncoders() {

//        for (DcMotor motor : driveMotors) {
//             motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startDriveEncoderless() {
        // DISABLED FOR LIFTER TESTING
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startDriveEncodersTarget() {

//        for (DcMotor motor : driveMotors) {
//             motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //Additonal robot functions


    public void GyroRotateDEGTele(int maxDegrees, double power, float angle) {
        // IMU output is positive for left turn and negative for right turn.  Max degrees determines direction.
        //angle needs to be positive for left turn and negative for right turn as off 3-19-2
        //

        //   imu.initialize(parameters);   No need to initialize here as it sets the teh zero to teh worng angle
        // IMU initialization is done in power shot.  It takes along time >1-sec

        boolean turnRight;  // flag to check rotation direction
        boolean turnComplete = false;   // flag while loop

        // conversion for ticks to ticks
        final double conversion_factor = 12.73;

        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;

        // Check which direction to turn robot and adjust drive motor power direction
        if (angle < 0) {
            turnRight = true;
        } else {
            turnRight = false;
            power = power * -1;
        }

        // drive with encoders slows the IMU down and it is very inaccurate.  Use run without encoders here

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // start motion.
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and all motors are running.
        //while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
        while (!turnComplete) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (angles.firstAngle <= 0 && turnRight) {
                turnComplete = true;
                //break;

            } else if (angles.firstAngle >= 0 && !turnRight) {
                turnComplete = true;
                //break;
            }
        }
        stopDriveMotors();

    } //end RotateDegTele

    // gets current angle from imu
    public float getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    //*************************************************************************************************

    // Lifter

    // TeleOp lifter functions

    public void asynchLift(double ticks, double power, LinearOpMode linearOpMode) {
        double lifterposition = lifter.getCurrentPosition();
        if (power < 0) {
            ticks = -ticks;
        }
        double target = lifterposition + ticks;
        lifter.setTargetPosition((int) target);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(power);
    }
    public void absoluteasynchLift(int ticks, double power, LinearOpMode linearOpMode) {
        lifter.setTargetPosition(ticks);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(power);
    }

    public void asynchLiftNoL(double ticks, double power) {
        double lifterposition = lifter.getCurrentPosition();
        if (power < 0) {
            ticks = -ticks;
        }
        double target = lifterposition + ticks;
        lifter.setTargetPosition((int) target);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(power);
    }
    public void absoluteasynchLiftNoL(int ticks, double power) {
        lifter.setTargetPosition(ticks);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(power);
    }
    //lifter.isBusy();

    // Auto lifter functions

    // movement method for lifter in autonomous
    // ticks should always be positive and the power will set the direction
    public void lifterA(double ticks, double power, LinearOpMode linearOpMode) {
        double lifterposition = lifter.getCurrentPosition();
        if (power < 0) {
            ticks = -ticks;
        }
        double target = lifterposition + ticks;
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setPower(power);
        if (power > 0) {
            while (linearOpMode.opModeIsActive() && lifterposition < target) {
                lifterposition = lifter.getCurrentPosition();
                systemTools.telemetry.addData("counts", lifterposition);
                systemTools.telemetry.update();
            }
        } else {
            while (linearOpMode.opModeIsActive() && lifterposition > target) {
                lifterposition = lifter.getCurrentPosition();
            }
        } // treats 0 power as negative but it wont do anything anyway

        lifter.setPower(0);
    }


//**********************************************************************************************
    // opencv recognition methods

//    public void activateConeCam() {
//        pipeline = new PowerPlayPipeline(true, ColorVals.HUE_MIN, ColorVals.HUE_MAX, ColorVals.SATURATION_MIN, ColorVals.SATURATION_MAX, ColorVals.VALUE_MIN, ColorVals.VALUE_MAX);
//
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 2")); // was webcam 1 1-25-23
//        camera.setPipeline(pipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//    }


}
