// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class UtilitiesFieldSectioning {
    //scoring positions - less accurate - most likely going to use sections rather than this
    private final Pose2d L1 = new Pose2d(5.359,5.559,new Rotation2d().fromDegrees(0));
    private final Pose2d L2 = new Pose2d();
    private final Pose2d L3 = new Pose2d();
    private final Pose2d L4 = new Pose2d();
    private final Pose2d L5 = new Pose2d();
    private final Pose2d L6 = new Pose2d();
    private final Pose2d R1 = new Pose2d();
    private final Pose2d R2 = new Pose2d();
    private final Pose2d R3 = new Pose2d();
    private final Pose2d R4 = new Pose2d();
    private final Pose2d R5 = new Pose2d();
    private final Pose2d R6 = new Pose2d();

    //sections
    private static final Pose2d S1 = new Pose2d(5.359,5.559,new Rotation2d().fromDegrees(0));
    private static final Pose2d S2 = new Pose2d(6.533,4.169,new Rotation2d().fromDegrees(0));
    private static final Pose2d S3 = new Pose2d(5.454,2.539,new Rotation2d().fromDegrees(0));
    private static final Pose2d S4 = new Pose2d(3.333,2.563,new Rotation2d().fromDegrees(0));
    private static final Pose2d S5 = new Pose2d(2.493,4.049,new Rotation2d().fromDegrees(0));
    private static final Pose2d S6 = new Pose2d(3.488,5.428,new Rotation2d().fromDegrees(0));

    //array of positions
    public static final Pose2d[] sectionsArr = {S1, S2, S3, S4, S5, S6};

    /***
     * 
     * @param currentPose
     * @return closest section's pose
     */
    public static Pose2d getClosestSection(Pose2d currentPose) {
        Pose2d currentClosest = new Pose2d();
        double currentDistanceFromPoint = 999999; //set high so that no element is auto selected - will probably delete later
        double minDistance = currentDistanceFromPoint;
        for (int i = 0; i < sectionsArr.length; ++i) {
            // d = √(x2 - x1)2 + (y2 - y1)2
            currentDistanceFromPoint = Math.sqrt(Math.pow(sectionsArr[i].getX() - currentPose.getX(),2) + Math.pow(sectionsArr[i].getY() - currentPose.getY(), 2));
            if (currentDistanceFromPoint < minDistance) {
                minDistance = currentDistanceFromPoint;
                currentClosest = sectionsArr[i];
            }
        }
        return currentClosest;
    }
}
