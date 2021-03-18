package org.firstinspires.ftc.teamcode.sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;

@Disabled
public class SampleTrigAngleAdjustment extends LinearOpMode {
    Pose2d poseEstimate;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(0);

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.a) {
                drive.turnAsync(adjustmentDegs());
            }
        }
    }

    public double adjustmentDegs() {
        // A is along the Y axis (RoadRunner Coordinate System)
        // Y value can return same angle when positive with conditional at end
        double diffA = 72 - Math.abs(poseEstimate.getY());
        // B is along the X axis (RoadRunner Coordinate System)
        double diffB;

        // Accounting for negative X values
        if (poseEstimate.getX() < 0) {
            diffB = 72 + (72 - Math.abs(poseEstimate.getX()));
        } else {
            // For positive X values
            diffB = 72 - poseEstimate.getX();
        }

        // Find angle with atan2 of A over B
        double adjustment = Math.toDegrees(Math.atan2(diffA, diffB));

        // For making sure turning is the right way
        if (poseEstimate.getY() > 0) {
            adjustment = -(adjustment);
        } else {
            adjustment = Math.abs(adjustment);
        }

        // For RoadRunner criteria
        return Math.toRadians(adjustment);
    }

}
