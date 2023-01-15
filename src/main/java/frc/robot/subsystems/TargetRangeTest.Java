/*public class Robot extends TimedRobot {
    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // PID constants should be tuned per robot
    final double P_GAIN = 0.1;
    final double D_GAIN = 0.0;
    PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

    XboxController xboxController;

    // Drive motors
    PWMVictorSPX leftMotor = new PWMVictorSPX(0);
    PWMVictorSPX rightMotor = new PWMVictorSPX(1);
    DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    @Override
    public void robotInit() {
        xboxController = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        double forwardSpeed;
        double rotationSpeed = xboxController.getLeftX();

        if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);
            } else {
                // If we have no targets, stay still.
                forwardSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            forwardSpeed = -xboxController.getRightY();
        }

        // Use our forward/turn speeds to control the drivetrain
        drive.arcadeDrive(forwardSpeed, rotationSpeed);
    }
}*/