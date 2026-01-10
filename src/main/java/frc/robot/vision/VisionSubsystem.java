// package frc.robot.vision;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.drivetrain.CommandSwerveDrivetrain;

// import org.photonvision.EstimatedRobotPose;

// import java.util.Optional;

// public class VisionSubsystem extends SubsystemBase {
//     public CommandSwerveDrivetrain drivetrain;
//     AprilTagFieldLayout fieldLayout =
// AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

//     Camera frontCamera =
//             new Camera(
//                     "front",
//                     new Transform3d(
//                             new Translation3d(
//                                     Units.inchesToMeters(13.0),
//                                     Units.inchesToMeters(5.0),
//                                     Units.inchesToMeters(6.0)),
//                             new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(0))));
//     Camera backCamera =
//             new Camera(
//                     "back",
//                     new Transform3d(
//                             new Translation3d(
//                                     Units.inchesToMeters(13.25),
//                                     Units.inchesToMeters(-2.25),
//                                     Units.inchesToMeters(4.75)),
//                             new Rotation3d(0.0, 0.0, Math.toRadians(160))));

//     Camera[] cameras = {frontCamera, backCamera};

//     public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
//         this.drivetrain = drivetrain;
//     }

//     @Override
//     public void periodic() {
//         for (Camera camera : cameras) {
//             Optional<EstimatedRobotPose> estimatedPose = camera.update();
//             if (estimatedPose.isPresent()) {
//                 Pose2d pose = estimatedPose.get().estimatedPose.toPose2d();
//                 double timestamp = estimatedPose.get().timestampSeconds;
//                 drivetrain.addVisionMeasurement(pose, timestamp);
//             }

//             System.out.println("1: " + fieldLayout.getTagPose(1));
//             System.out.println("11: " + fieldLayout.getTagPose(11));
//         }
//     }

//     @Override
//     public void initSendable(SendableBuilder builder) {}
// }
