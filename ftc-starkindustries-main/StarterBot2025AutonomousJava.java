package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

// Register this as an autonomous OpMode for the robot controller app
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="StarterBot_V1_Autonomous", group="Linear Opmode")
public class StarterBot2025AutonomousJava extends LinearOpMode {

    // Declare hardware variables for robot components
    private DcMotor leftDrive = null;  // Left drive motor for movement
    private DcMotor rightDrive = null; // Right drive motor for movement
    private DcMotor arm = null;        // Motor for moving the robot's arm
    private DcMotor wrist = null;      // Motor for moving the wrist attached to the arm
    private Servo claw = null;         // Servo for controlling the claw's open/close state
    private CRServo intake = null;     // Continuous rotation servo for handling intake actions

    // Arm positions corresponding to specific tasks
    private static final int ARM_POSITION_INIT = 300;       // Arm position for initialization
    private static final int ARM_POSITION_INTAKE = 450;     // Arm position for picking up objects
    private static final int ARM_POSITION_HOVER_HIGH = 2600;// Arm position for hovering at a high position

    // Wrist positions for specific orientations
    private static final int WRIST_POSITION_INIT = 0;       // Wrist position for initialization
    private static final int WRIST_POSITION_SAMPLE = 270;   // Wrist position for sampling or reaching

    // Claw positions for open and closed states
    private static final double CLAW_OPEN_POSITION = 0.55;  // Open claw position
    private static final double CLAW_CLOSED_POSITION = 0.7; // Closed claw position

    @Override
    public void runOpMode() {
        // Map hardware components to configuration names in the Robot Controller app
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");

        // Configure motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);  // Reverse left motor for correct forward motion
        rightDrive.setDirection(DcMotor.Direction.FORWARD); // Forward direction for right motor

        // Reset encoder positions for precise motion tracking
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the start signal from the driver station
        waitForStart();

        // Check if the OpMode is active (game has started)
        if (opModeIsActive()) {
            // Execute a series of autonomous actions
            moveArm(ARM_POSITION_INIT, WRIST_POSITION_INIT); // Move arm and wrist to initial positions
            openClaw();                                      // Open the claw
            moveForward(1000, 0.5);                          // Drive forward for 1 second at 50% power
            closeClaw();                                     // Close the claw to grab an object
            moveArm(ARM_POSITION_HOVER_HIGH, WRIST_POSITION_SAMPLE); // Move arm to high hover position
            moveBackward(1000, 0.5);                         // Drive backward for 1 second at 50% power
        }
    }

    /**
     * Move the arm and wrist to specified positions.
     * @param armTarget The target encoder position for the arm motor.
     * @param wristTarget The target encoder position for the wrist motor.
     */
    private void moveArm(int armTarget, int wristTarget) {
        arm.setTargetPosition(armTarget);  // Set target position for the arm
        wrist.setTargetPosition(wristTarget); // Set target position for the wrist
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Use encoders to move to position
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1.0);  // Set arm motor power to full
        wrist.setPower(1.0); // Set wrist motor power to full

        // Wait until both motors reach their target positions
        while (arm.isBusy() || wrist.isBusy()) {
            // Send telemetry data to the driver station for debugging
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Wrist Position", wrist.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors after reaching the target
        arm.setPower(0);
        wrist.setPower(0);
    }

    /**
     * Open the claw to release objects.
     */
    private void openClaw() {
        claw.setPosition(CLAW_OPEN_POSITION); // Move servo to open position
        sleep(500); // Wait for servo to complete the motion
    }

    /**
     * Close the claw to grab objects.
     */
    private void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POSITION); // Move servo to closed position
        sleep(500); // Wait for servo to complete the motion
    }

    /**
     * Drive the robot forward for a specified duration and power.
     * @param duration Time to drive in milliseconds.
     * @param power Motor power level (0 to 1).
     */
    private void moveForward(long duration, double power) {
        leftDrive.setPower(power);  // Set power for left motor
        rightDrive.setPower(power); // Set power for right motor
        sleep(duration); // Drive for the specified duration
        stopMotors(); // Stop all motors
    }

    /**
     * Drive the robot backward for a specified duration and power.
     * @param duration Time to drive in milliseconds.
     * @param power Motor power level (0 to 1).
     */
    private void moveBackward(long duration, double power) {
        leftDrive.setPower(-power);  // Reverse left motor
        rightDrive.setPower(-power); // Reverse right motor
        sleep(duration); // Drive for the specified duration
        stopMotors(); // Stop all motors
    }

    /**
     * Stop all drive motors.
     */
    private void stopMotors() {
        leftDrive.setPower(0);  // Stop left motor
        rightDrive.setPower(0); // Stop right motor
    }
}
