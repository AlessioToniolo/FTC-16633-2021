package org.firstinspires.ftc.teamcode.competition;

// Import statements
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utility.MaxStaticVelocity;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;
import org.firstinspires.ftc.teamcode.utility.external.TuningController;

@Autonomous(group = "competition")
public class CompetitionAuto extends LinearOpMode {

    // Global access to the drive base
    SampleMecanumDrive drive;

    // Global access to the start pose
    Pose2d startPose;

    // Global access to the subassemblies' motors and servos
    DcMotor wobbleArm;
    DcMotorEx shooter;
    double encoderVelo = 0.0;
    public static PIDCoefficients PID = new PIDCoefficients(14,0,10);
    Servo wobbleServo;
    Servo shooterServo;
    DcMotor frontRoller;
    DcMotor bottomRoller;

    // For setting a clock in the opmode
    private ElapsedTime runtime = new ElapsedTime();

    // Constants
    final double COUNTS_PER_DEGREE = 4;

    // Tensorflow variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    // API key in order to get access to the vuforia library
    private static final String VUFORIA_KEY =
                    "AcpsyBb/////AAABmU29bF3HdkvIocXaucsm3TJpvnPnJLn/FLfZMsVu3E7dEtnQaxTDq0uGZev7uB3edznX33u" +
                            "M8wIzZ5Q+eCe/CPKmw+SZzsBw6k3IDh76mtYpnDlqTHPwlheJ236xzf8CaC91sXjGRliLaEYdvr+JztgAZe" +
                            "wY5vUd4+aYGBZZLAU/4Ur4RpvjwhNuy9z78K5In4R4T+WoikzFRIW27prJv9veYWOKTYZFj8xMsj+4nORRDSNXiYZ" +
                            "pv7nG9cdOO7Pqvv+ZeaMZCLYTEv1ZL8yT8NyRP+kBPFpdmBYr2DW6uZmv8bUTJ68ZG1IEQ2Pt3bLeP6bWWy3Z0B" +
                            "hYNdButQ8Zt3Y9xINvVtEjVkHQZ5ovquV/";

    // The instance of the vuforia localilzer engine that the partly usesopmode uses
    private VuforiaLocalizer vuforia;

    // The instance of the tensorflow object detection engine that the opmode uses
    private TFObjectDetector tfod;

    // The method where ALL of the robot code will run
    @Override
    public void runOpMode() throws InterruptedException // To allow for emergency stops
    {

        // Our drive base robot class
        drive = new SampleMecanumDrive(hardwareMap);

        // Where the robot starts (RIGHT RED LINE for all target zones)
        startPose = new Pose2d(-63, -50, Math.toRadians(0));
        // Setting the first pose to where we start
        drive.setPoseEstimate(startPose);

        // Initializing our subassemblies' motors and servos
        wobbleArm = hardwareMap.dcMotor.get("wobblearm");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        wobbleServo = hardwareMap.servo.get("wobbleservo");
        shooterServo = hardwareMap.servo.get("shooterservo");
        frontRoller = hardwareMap.dcMotor.get("frontroller");
        bottomRoller = hardwareMap.dcMotor.get("bottomroller");

        // Setting the modes
        frontRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first
        initVuforia();
        initTfod();

        // Activate TensorFlow Object Detection before we wait for the start command
        // We do it here so that the Camera Stream window will have the TensorFlow annotations visible
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).
            tfod.setZoom(1.5, 1.78); // 2.5, 1.78
        }

        // Wait for the game to begin
        // TODO: DO NOT PLAY UNTIL YOU SEE THIS TELEMETRY
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // The variable that holds the amount of rings in the starter stack
        int count = 0;

        // The variable that holds which target zone
        String label = "None";

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (count > 120) {
                    count += 1;
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                label = recognition.getLabel();
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }

        // To disable tfod in order to run rest of opmode if tfod is done detecting
        if (tfod != null) {
            tfod.shutdown();
        }

        // Run each method based on the ring detection
        if (label.equals("None")) {
            targetA();
        } else if (label.equals("Single")) {
            targetB();
        } else if (label.equals("Quad")) {
            targetC();
        } else {
            targetA();
        }

        // Test output and detection
        telemetry.addData("Final Label", label);
        telemetry.update();
    }


    // Initialize the Vuforia localization engine
    private void initVuforia() {

        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine
    }

    // Initialize the TensorFlow Object Detection engine
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    // Delay Code for timing with the sub assemblies
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    // For moving the wobble arm to a certain position
    public void wobbleArmDegSet(double speed, double deg, double timeoutS){
        int target;

        deg = deg * 2;
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        target = (int)(deg * COUNTS_PER_DEGREE);
        wobbleArm.setTargetPosition(target);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        wobbleArm.setPower(Math.abs(speed));
    }

    // PIDF shooting function
    public void setBasicVelocity(double power) {
        encoderVelo = -((power * (100)) * 28);
        shooter.setVelocityPIDFCoefficients(PID.p, PID.i, PID.d, MaxStaticVelocity.maxStaticPIDFVelocity);
        shooter.setVelocity(encoderVelo);
    }

    public void shooterOn(double power) { setBasicVelocity(power); }

    public void terminateShooter() { shooter.setPower(0); }

    public void shootRing(double speed, int times) {
        shooterOn(speed);
        delay(0.3);
        for (int i = 0; i < times; i++) {
            flick();
        }
    }

    public void flick() {
        shooterServo.setPosition(0.2);
        delay(0.6);
        shooterServo.setPosition(0.05);
    }
    public void wobbleManipulation(boolean open) {
        if (open) {
            wobbleServo.setPosition(0.5);
        } else { wobbleServo.setPosition(0.85); }
    }

    // Target zone functions

    // Target A (0 rings)
    public void targetA() {
        // TODO: sample movement for now, not real autonomous movement
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-7.5, -60.0), 0.0)
                .build();
        drive.followTrajectory(traj1);
        drive.turnAsync(Math.toRadians(45));

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    // Target A (1 rings)
    public void targetB() {

    }

    // Target A (4 rings)
    public void targetC() {

    }
}

