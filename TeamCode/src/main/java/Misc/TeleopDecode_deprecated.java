/*
Documentation:
-insert-useful info here
- Plug in usb-a in computer and a usb-c out to the robot's control hub
*/
package Misc;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Decode Teleop")

public class TeleopDecode_deprecated extends OpMode {

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor turretMotor; //The motor that launches the ball
    public Servo servo1;
    public CRServo servo2;

    @Override
    public void init() { //This runs when hitting the init button on the driver station

        telemetry.addData("Hardware: ","Initialized");

        //Hardware mapping gives each hardware object a name, which must be entered in the robot's configuration
        //Motor Mapping
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor = hardwareMap.get(DcMotor.class,"slideMotor");

        //Servo Mapping
        servo1 = hardwareMap.get(Servo.class,"servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        //Setting motors to run using encoders
        DcMotor[] motors = {frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor};
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void loop() { //This repeats when you hit the start button
        mecanum_drivetrain();
        //motor_test();
        controls_telemetry();
    }

    public void motor_test() {
        frontLeftMotor.setPower(gamepad2.left_stick_y);
        frontRightMotor.setPower(gamepad2.right_stick_y);
        backLeftMotor.setPower(gamepad2.left_stick_x);
        backRightMotor.setPower(gamepad2.right_stick_x);
    }

    public void mecanum_drivetrain() { //Checks joystick input and accordingly sets power level to motors in the mecanum drivetrain

        /*Joystick variables and denominator (this is the band-aid solution joystick mapping)
        double y = -gamepad2.right_stick_x;
        double x = gamepad2.right_stick_y * 1.1;
        double r = gamepad2.left_stick_y;
        */

        //(This is the original mecanum joystick mapping)
        double y = gamepad2.left_stick_y;
        double x = -gamepad2.left_stick_x * 1.1;
        double r = -gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        //Power variables calculated from joystick variables and denominator
        double frontLeftPower = (y + x + r) / denominator;
        double frontRightPower = (y - x - r) / denominator;
        double backLeftPower = (y - x + r) / denominator;
        double backRightPower = (y + x -r) / denominator;

        //Setting power ot motors using the power variables
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void controls_telemetry(){
        telemetry.addData("triangle: ",gamepad2.triangle);
        telemetry.addData("cross: ",gamepad2.cross);
        telemetry.addData("square: ", gamepad2.square);
        telemetry.addData("circle: ",gamepad2.circle);
        telemetry.addData("left_bumper: ",gamepad2.left_bumper);
    }
}