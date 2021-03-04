package org.firstinspires.ftc.teamcode.sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;

public class SampleLUTPowerAdjustment extends LinearOpMode {

    // LUT
    InterpLUT lut = new InterpLUT();
        // TODO: add keys
        // ex: `add(60.0, 0.5);`
    // lut.createLUT();

    // Current pose
    Pose2d poseEstimate;

    // Hypot distance from goal
    double hypotDistance;

    // Vector distance from goal
    Vector2d newVec;

    // Target vector of the goal (for right side)
    Vector2d TARGET_VEC = new Vector2d(72, -36);

    // Shooter motor
    DcMotor flywheel = hardwareMap.dcMotor.get("flywheel");
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
            telemetry.addData("x", poseEstimate.getX());

            // Hypotenuse math
            newVec = TARGET_VEC.minus(drive.getPoseEstimate().vec());
            hypotDistance = Math.hypot(newVec.getX(), newVec.getY());

            flywheel.setPower(lut.get(hypotDistance));
        }
    }
}
