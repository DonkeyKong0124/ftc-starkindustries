package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Enhanced: 4-Motor Omni Drive with Dual Forearms and Claw", group="Linear OpMode")
public class BasicOmniTeleOpMode extends LinearOpMode {

    // Declare OpMode members for motors and servos.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor towerarmMotor = null;
    private DcMotor foreArm = null; // Declare REV Core Hex Motor here
    private Servo intakeServo = null;
    private Servo clawServo = null;
    private boolean clawopen = true;
    private boolean lastBump = false;
    

    @Override
    public void runOpMode() {

        // Initialize hardware variables with names matching the robot configuration.
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_rear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_rear");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        towerarmMotor = hardwareMap.get(DcMotor.class, "tower_arm");
        foreArm = hardwareMap.get(DcMotor.class, "fore_arm"); // Initialize the Core Hex Motor
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        // Set motor directions.
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        towerarmMotor.setDirection(DcMotor.Direction.FORWARD);
        foreArm.setDirection(DcMotor.Direction.FORWARD); // Set Core Hex Motor direction

        // Display status and update telemetry.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Main loop
        while (opModeIsActive()) {

            // Control for forward/backward using triggers.
            double forwardPower = gamepad1.right_trigger - gamepad1.left_trigger;

            // Control for strafing using D-pad.
            double strafePower = 0.0;
            if (gamepad1.dpad_right) {
                strafePower = 1.0; // Right strafing
            } else if (gamepad1.dpad_left) {
                strafePower = -1.0; // Left strafing
            }

            // Combine driving and strafing controls.
            double leftFrontPower = forwardPower + strafePower;
            double rightFrontPower = forwardPower - strafePower;
            double leftBackPower = forwardPower - strafePower;
            double rightBackPower = forwardPower + strafePower;

            // Set motor power for drivetrain.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Tower arm control: Move up/down with Y-axis of the left joystick.
            double towerarmMotorPower = -gamepad1.left_stick_y; // Y-axis controls tower arm
            towerarmMotor.setPower(towerarmMotorPower);

            // Forearm control: Move up/down with X-axis of the left joystick.
            double foreArmMotorPower = gamepad1.left_stick_x; // X-axis controls forearm
            foreArm.setPower(foreArmMotorPower);

            // Control for intake servo using RB and LB buttons.
            if (gamepad1.right_bumper) {
                intakeServo.setPosition(1.0); // Rotate servo forward
            } else if (gamepad1.left_bumper) {
                intakeServo.setPosition(0.0); // Rotate servo backward
            } else {
                intakeServo.setPosition(0.5); // Stop the servo (neutral position)
            }

            // Control for claw servo using X and Y buttons.
            if (gamepad1.x && !clawopen) {
                clawopen = !clawopen;
                if (clawopen) {
                    clawServo.setPosition(0.55); // Open claw
                } else {
                    clawServo.setPosition(0.7); // Close claw
                }
            }
            lastBump = gamepad1.x;

            // Display telemetry data.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "LF (%.2f), RF (%.2f), LB (%.2f), RB (%.2f)", 
                leftFrontDrive.getPower(), rightFrontDrive.getPower(), leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.addData("Towerarm Power", "%.2f", towerarmMotor.getPower());
            telemetry.addData("Fore Arm Power", "%.2f", foreArm.getPower()); // Telemetry for Core Hex Motor
            telemetry.addData("Intake Servo Position", "%.2f", intakeServo.getPosition());
            telemetry.addData("Claw Servo Position", "%.2f", clawServo.getPosition());
            telemetry.update();
        }
    }
}
