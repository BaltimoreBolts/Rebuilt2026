package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNav questNav = new QuestNav();

  // SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDrivetrain();
  // Matrix<N3, N1> QUESTNAV_STD_DEVS =
  //     VecBuilder.fill(
  //         0.02, // Trust down to 2cm in X direction
  //         0.02, // Trust down to 2cm in Y direction
  //         0.035 // Trust down to 2 degrees rotational
  //     );

  public void getRobotPose() {
    // First, Declare our geometrical transform from the robot center to the Quest
    // Transform3d ROBOT_TO_QUEST = new Transform3d( /*TODO: Put your x, y, z, yaw, pitch, and roll
    // offsets here!*/ );

    // // Get the latest pose data frames from the Quest
    // PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    // if (poseFrames.length > 0) {
    //     // Get the most recent Quest pose
    //     Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();

    //     // Transform by the mount pose to get your robot pose
    //     Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
    // }
  }

  public void setRobotPost() {
    // // First, Declare our geometrical transform from the robot center to the Quest
    // Transform3d ROBOT_TO_QUEST = new Transform3d( /*TODO: Put your x, y, z, yaw, pitch, and roll
    // offsets here!*/ );

    // // Assume this is the requested reset pose
    // Pose3d robotPose = new Pose3d( /* Some pose data */ );

    // // Transform by the offset to get the Quest pose
    // Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);

    // // Send the reset operation
    // questNav.setPose(questPose);
  }

  @Override
  public void periodic() {
    // Get the latest pose data frames from the Quest
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    // Loop over the pose data frames and send them to the pose estimator
    // for (PoseFrame questFrame : questFrames) {
    //     // Make sure the Quest was tracking the pose for this frame
    //     if (questFrame.isTracking()) {
    //         // Get the pose of the Quest
    //         Pose3d questPose = questFrame.questPose3d();
    //         // Get timestamp for when the data was sent
    //         double timestamp = questFrame.dataTimestamp();

    //         // Transform by the mount pose to get your robot pose
    //         Pose3d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

    //         // You can put some sort of filtering here if you would like!

    //         // Add the measurement to our estimator
    //         swerveDriveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp,
    // QUESTNAV_STD_DEVS);
    //     }
    // }
  }
}
