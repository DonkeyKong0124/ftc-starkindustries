package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Enhanced: Mecanum Drive with Forearm and Claw", group="Linear OpMode")
public class EnhancedMecanumTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor towerArmMotor = null;
    private DcMotor foreArmMotor = null;
    private Servo intakeServo = null;
    private Servo clawServo = null;
    private boolean clawOpen = true; 
    private boolean lastRBState = false; 

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_rear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_rear");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        towerArmMotor = hardwareMap.get(DcMotor.class, "tower_arm");
        foreArmMotor = hardwareMap.get(DcMotor.class, "fore_arm");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        // Set motor directions
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Drivetrain control
            double forward = -gamepad1.left_stick_y;  // Forward/Backward
            double yaw = gamepad1.left_stick_x;       // Yaw (Rotation)
            double strafe = gamepad1.right_stick_x;   // Strafing (Side-to-side)

            double leftFrontPower = forward + strafe + yaw;
            double rightFrontPower = forward - strafe - yaw;
            double leftBackPower = forward - strafe + yaw;
            double rightBackPower = forward + strafe - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Tower arm control (D-pad Up/Down)
            if (gamepad1.dpad_up) {
                towerArmMotor.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                towerArmMotor.setPower(-0.5);
            } else {
                towerArmMotor.setPower(0);
            }

            // Forearm control (D-pad Left/Right)
            if (gamepad1.dpad_left) {
                foreArmMotor.setPower(-0.5);
            } else if (gamepad1.dpad_right) {
                foreArmMotor.setPower(0.5);
            } else {
                foreArmMotor.setPower(0);
            }

            // Intake control (Triggers)
            if (gamepad1.right_trigger > 0.1) {
                intakeServo.setPosition(1.0); // Rotate forward
            } else if (gamepad1.left_trigger > 0.1) {
                intakeServo.setPosition(0.0); // Rotate backward
            } else {
                intakeServo.setPosition(0.5); // Neutral
            }

            // Claw control (RB button)
            if (gamepad1.right_bumper && !lastRBState) {
                clawOpen = !clawOpen;
                clawServo.setPosition(clawOpen ? 0.6 : 0.7); // Adjust positions as needed
            }
            lastRBState = gamepad1.right_bumper;

            // Telemetry for debugging
            telemetry.addData("Claw Open", clawOpen);
            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }
}
