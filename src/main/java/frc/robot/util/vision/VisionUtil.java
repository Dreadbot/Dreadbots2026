package frc.robot.util.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionUtil {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    /**
     * 
     * @param tagId ID of tag
     * @return Pose3d of apriltag position
     */
    public static Pose3d getApriltagPose(int tagId) {
        return FIELD_LAYOUT.getTagPose(tagId).get();
    }

    /**
     * Converts Tag Pose to world coordinates 
     * @param offset Pose of Tag of from robot coorindate frame
     * @param tagPose Pose of tag in world frame
     * @return Pose converted to world axes
    
    */
    public static Pose2d tagAxisToWorldAxis(Pose3d offset, Pose3d tagPose) {
        return new Pose2d(offset.getTranslation().rotateAround(tagPose.getTranslation(), tagPose.getRotation()).toTranslation2d(), offset.getRotation().toRotation2d()); // Convert to world axes + ignore Z travel
    }

    /**
     * Returns whether the an AprilTag is on the HUB based on its ID
     * @param id ID of tag
     * @return True if the tag is on the HUB
     */

    public static boolean isHubId(int id) {
        return
           id == 2
        || id == 3
        || id == 4
        || id == 5
        || id == 8
        || id == 9
        || id == 10
        || id == 11
        || id == 18
        || id == 19
        || id == 20
        || id == 21
        || id == 24
        || id == 25
        || id == 26
        || id == 27;
    }
}