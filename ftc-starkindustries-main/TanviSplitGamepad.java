package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TanviSplitGamepad", group = "Linear OpMode")
public class TanviSplitGamepad extends LinearOpMode {

    // Declare hardware variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontWheel = null;
    private DcMotor rightRearWheel = null;
    private DcMotor leftFrontWheel = null;
    private DcMotor leftRearWheel = null;

    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;
    private DcMotor horizontalSlide = null;

    private Servo verticalClaw = null;
    private Servo horizontalClaw = null;

    @Override
    public void runOpMode() {

        // Initialize hardware variables
        rightFrontWheel = hardwareMap.get(DcMotor.class, "frontRight");
        rightRearWheel = hardwareMap.get(DcMotor.class, "backRight");
        leftFrontWheel = hardwareMap.get(DcMotor.class, "frontLeft");
        leftRearWheel = hardwareMap.get(DcMotor.class, "backLeft");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "linearSlideLeft");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "linearSlideRight");
        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");

        verticalClaw = hardwareMap.get(Servo.class, "verticalClaw");
        horizontalClaw = hardwareMap.get(Servo.class, "horizontalClaw");

        // Set motor directions
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheel.setDirection(DcMotor.Direction.FORWARD);
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheel.setDirection(DcMotor.Direction.REVERSE);

        leftLinearSlide.setDirection(DcMotor.Direction.REVERSE);
        rightLinearSlide.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Main control loop
        while (opModeIsActive()) {

            // ---------------- GAMEPAD 1: DRIVING ---------------- //
            double axialPower = -gamepad1.left_stick_y;    // Forward/Backward
            double lateralPower = gamepad1.left_stick_x / 2; // Strafing
            double yawPower = gamepad1.right_stick_x;     // Rotation

            rightFrontWheel.setPower(axialPower - lateralPower - yawPower);
            rightRearWheel.setPower(-axialPower + lateralPower - yawPower);
            leftFrontWheel.setPower(axialPower + lateralPower + yawPower);
            leftRearWheel.setPower(-axialPower - lateralPower + yawPower);

            // ---------------- GAMEPAD 2: LINEAR SLIDES ---------------- //
            if (gamepad2.dpad_up) {
                leftLinearSlide.setTargetPosition(-500);
                rightLinearSlide.setTargetPosition(-500);
            }

            double verticalUpPower = gamepad2.right_stick_y;
            leftLinearSlide.setPower(-verticalUpPower);
            rightLinearSlide.setPower(-verticalUpPower);

            double verticalDownPower = gamepad2.left_trigger;
            leftLinearSlide.setPower(10 * verticalDownPower);
            rightLinearSlide.setPower(10 * verticalDownPower);

            // ---------------- GAMEPAD 2: HORIZONTAL SLIDE ---------------- //
            double horizontalPower = gamepad2.right_stick_y;
            horizontalSlide.setPower(horizontalPower);

            // ---------------- GAMEPAD 2: CLAW CONTROLS ---------------- //
            // Vertical Claw
            if (gamepad2.b) {
                verticalClaw.setPosition(0.1); // Open claw
            }
            if (gamepad2.x) {
                verticalClaw.setPosition(1.0); // Close claw
            }

            // Horizontal Claw
            if (gamepad2.y) {
                horizontalClaw.setPosition(0.0); // Open claw
            }
            if (gamepad2.a) {
                horizontalClaw.setPosition(1.0); // Close claw
            }

            // ---------------- TELEMETRY ---------------- //
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Linear Slide Position", leftLinearSlide.getCurrentPosition());
            telemetry.addData("Right Linear Slide Position", rightLinearSlide.getCurrentPosition());
            telemetry.addData("Horizontal Slide Power", horizontalPower);
            telemetry.addData("Vertical Claw Position", verticalClaw.getPosition());
            telemetry.addData("Horizontal Claw Position", horizontalClaw.getPosition());
            telemetry.update();
        }
    }
}
