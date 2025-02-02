// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

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
    private static final Pose2d S1 = new Pose2d(5.359,5.559,new Rotation2d().fromDegrees(48.752));
    private static final Pose2d S2 = new Pose2d(6.533,4.169,new Rotation2d().fromDegrees(4.135));
    private static final Pose2d S3 = new Pose2d(5.454,2.539,new Rotation2d().fromDegrees(-63.682));
    private static final Pose2d S4 = new Pose2d(3.333,2.563,new Rotation2d().fromDegrees(-122.367));
    private static final Pose2d S5 = new Pose2d(2.493,4.049,new Rotation2d().fromDegrees(-178.152));
    private static final Pose2d S6 = new Pose2d(3.488,5.428,new Rotation2d().fromDegrees(120.208));
    private static final Pose2d F1 = new Pose2d(1.858, 6.590,new Rotation2d().fromDegrees(-48.832) ); //feed station
    //pid
    private static final ProfiledPIDController angleController = new ProfiledPIDController(DriveCommands.ANGLE_KP,0, DriveCommands.ANGLE_KD, new Constraints(DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));
    

    
        //array of positions
        public static final Pose2d[] sectionsArr = {S1, S2, S3, S4, S5, S6,F1};
        
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
        public static String getClosestSectionName(Pose2d currentPose) {
            Pose2d closest = getClosestSection(currentPose);
            angleController.enableContinuousInput(-Math.PI, Math.PI);
            int value = 0;
            for (int i = 0; i < sectionsArr.length;++i) {
                if (sectionsArr[i] == closest) {
                    value = i;
                }
            }
            if (value <= 5) {
                return "S" + (value+1);
            }
            else {
                return "F" + (value - 5);
            }
        }
        public static void faceClosestReef(Pose2d currentPose, Drive drive) {
            Pose2d closest = getClosestSection(currentPose);
            angleController.enableContinuousInput(-Math.PI, Math.PI);
            double omega = angleController.calculate(currentPose.getRotation().getRadians(), closest.getRotation().getRadians());
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, -omega);
            boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getRotation());
            drive.runVelocity(speeds);
    }
}
