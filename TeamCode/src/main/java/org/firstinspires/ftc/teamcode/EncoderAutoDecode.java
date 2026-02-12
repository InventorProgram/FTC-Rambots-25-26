package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
Description: An autonomous routine relying on the encoders and the IMU. The initial version was AI-generated.
 */

@Autonomous(name = "DECODE Hardcoded Auto - No Vision", group = "StarterBot")
public class EncoderAutoDecode extends LinearOpMode {

    // Hardware from your teleop
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx leftLauncher, rightLauncher;
    private CRServo leftFeeder, rightFeeder;
    private Servo diverter;
    private IMU imu;

    // Constants from your teleop
    private final double LAUNCHER_TARGET = 1200;      // close goal
    private final double LAUNCHER_MIN    = 1175;
    private final double FEED_TIME       = 0.80;
    private final double FULL_SPEED      = 1.0;
    private final double STOP_SPEED      = 0.0;

    // Drive tuning - MEASURE THESE ON YOUR ROBOT!
    private final double TICKS_PER_INCH  = 42.8;      // ≈42-45 for goBILDA 4" mecanum + typical motor; tune!
    private final double AUTO_POWER      = 0.40;      // slow for accuracy
    private final double HEADING_P       = 0.015;     // simple proportional correction gain (tune 0.01-0.03)

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Hardware mapping (exact names from your teleop)
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        leftLauncher    = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher   = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder      = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder     = hardwareMap.get(CRServo.class, "rightFeeder");
        diverter        = hardwareMap.get(Servo.class, "diverter");

        // Directions & modes from your teleop
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");  // confirm name matches your config file

        // Recommended simple init
        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(myIMUparameters);

        // Reset heading to 0° (forward direction at start)
        imu.resetYaw();

        // Initial states
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        diverter.setPosition(0.15);  // middle-ish position; tune to your best fixed aim (0 to 0.2962)

        telemetry.addData("Status", "Initialized - encoders & IMU ready");
        telemetry.addData("TICKS_PER_INCH (tune!)", TICKS_PER_INCH);
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Reset everything at start
            resetEncoders();
            imu.resetYaw();  // Sets current heading to 0° (forward)

            // Step 1: Drive forward to shooting position (~30 inches forward; tune distance!)
            driveStraightInches(30.0, AUTO_POWER);

            // Step 2: Spin up launcher
            telemetry.addData("Action", "Spinning up launcher");
            telemetry.update();
            leftLauncher.setVelocity(LAUNCHER_TARGET);
            rightLauncher.setVelocity(LAUNCHER_TARGET);

            while (opModeIsActive() &&
                    (leftLauncher.getVelocity() < LAUNCHER_MIN || rightLauncher.getVelocity() < LAUNCHER_MIN)) {
                telemetry.addData("Left Vel", leftLauncher.getVelocity());
                telemetry.addData("Right Vel", rightLauncher.getVelocity());
                telemetry.update();
                sleep(50);
            }

            // Step 3: Shoot 3 times (adjust count based on preloads)
            for (int shot = 1; shot <= 3; shot++) {
                telemetry.addData("Shooting", shot);
                telemetry.update();

                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                sleep((long)(FEED_TIME * 1000));

                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);

                sleep(500);  // let velocity recover
            }

            // Step 4: Optional - back up slightly toward base or strafe to park
            driveStraightInches(-18.0, AUTO_POWER * 0.7);

            // Cleanup
            leftLauncher.setVelocity(0);
            rightLauncher.setVelocity(0);
            stopDrive();

            telemetry.addData("Auto", "Finished - Runtime: %.1f s", runtime.seconds());
            telemetry.update();
        }
    }

    // ---------------- Helpers ----------------

    private void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void driveStraightInches(double inches, double basePower) {
        if (Math.abs(inches) < 1.0) return;

        resetEncoders();  // reset per segment for better accuracy

        int targetTicks = (int) Math.round(Math.abs(inches) * TICKS_PER_INCH);
        int direction = (inches > 0) ? 1 : -1;

        // Set target positions (mecanum straight = same sign on all)
        leftFrontDrive.setTargetPosition(targetTicks * direction);
        rightFrontDrive.setTargetPosition(targetTicks * direction);
        leftBackDrive.setTargetPosition(targetTicks * direction);
        rightBackDrive.setTargetPosition(targetTicks * direction);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Drive with heading correction
        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opModeIsActive() &&
                (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                        leftBackDrive.isBusy() || rightBackDrive.isBusy())) {

            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = startHeading - currentHeading;  // positive = drifted right

            // Simple P correction: reduce left or right side slightly
            double correction = error * HEADING_P;

            double lfPower = basePower * direction - correction;
            double rfPower = basePower * direction + correction;
            double lbPower = basePower * direction - correction;
            double rbPower = basePower * direction + correction;

            // Clamp to avoid over-correction
            lfPower = Math.max(-1.0, Math.min(1.0, lfPower));
            rfPower = Math.max(-1.0, Math.min(1.0, rfPower));
            lbPower = Math.max(-1.0, Math.min(1.0, lbPower));
            rbPower = Math.max(-1.0, Math.min(1.0, rbPower));

            leftFrontDrive.setPower(lfPower);
            rightFrontDrive.setPower(rfPower);
            leftBackDrive.setPower(lbPower);
            rightBackDrive.setPower(rbPower);

            telemetry.addData("Heading Error", error);
            telemetry.update();
            sleep(20);
        }

        stopDrive();
        sleep(200);  // brief settle
    }
}