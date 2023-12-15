package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.ftccommon.configuration.RobotConfigResFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.RobotCenterStage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
public class TeleOpFieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //RobotCenterStage robot = new RobotCenterStage();

        double slow = 1.0;
        double lifterPower = 0;

        double power = 0;

        double lf = 0;
        double lr = 0;
        double rf = 0;
        double rr = 0;

        double turn = 0;

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            if(gamepad2.right_trigger != 0) {
                slow = 0.35;
            }
            else {
                slow = 1.0;
            }


            power = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
            lf    = Range.clip(power + turn, -1.0, 1.0) ;
            rf   = Range.clip(power - turn, -1.0, 1.0) ;

            drive.leftRear.setPower(lf);
            drive.rightRear.setPower(rf);
            drive.leftFront.setPower(lf);
            drive.rightFront.setPower(rf);

                // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
//            Vector2d input = new Vector2d(
//                    -gamepad1.left_stick_y*slow,
//                    -gamepad1.left_stick_x*slow
//            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            input.getX(),
//                            input.getY(),
//                            -gamepad1.right_stick_x
//                    )
//            );

            // Update everything. Odometry. Etc.
            drive.update();

            //servo stuff
            if(gamepad2.right_bumper){
                drive.elbow.setPosition(0);
            }
            if(gamepad2.left_bumper) {
                drive.elbow.setPosition(1);
            }
            if(gamepad2.a){
                drive.lifter.setTargetPosition(-50);
                drive.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.lifter.setPower(0.6);
                drive.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //intake Motor stuff

            lifterPower = gamepad2.right_stick_y;

            drive.intake.setPower(-gamepad2.left_stick_y);

            //lifter stuff
            if ((drive.lifter.getCurrentPosition() > -15)) {
                if(lifterPower < 0) {
                    drive.lifter.setPower(lifterPower);
                }
            }

            if(drive.lifter.getCurrentPosition() < -1000) {
                if(lifterPower > 0) {
                    drive.lifter.setPower(lifterPower);
                }
            }

            drive.lifter.setPower(gamepad2.right_stick_y);

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
        }
    }
}