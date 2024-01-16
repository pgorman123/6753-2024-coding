 // AprilTag constants - Contains locations and real positions of apriltags on the field. AprilTagVision.java references these.
 public static final double aprilTagWidth = Units.inchesToMeters(6.0);
 public static final AprilTagFieldLayout aprilTags =
     isWPIField
         ? new AprilTagFieldLayout(
             List.of(
                 new AprilTag(
                     1,
                     new Pose3d(
                         Units.inchesToMeters(610.125),
                         Units.inchesToMeters(43.5),
                         Units.inchesToMeters(19.25),
                         new Rotation3d(0.0, 0.0, Math.PI))),
                 new AprilTag(
                     2,
                     new Pose3d(
                         Units.inchesToMeters(610.375),
                         Units.inchesToMeters(109.5),
                         Units.inchesToMeters(19.25),
                         new Rotation3d(0.0, 0.0, Math.PI))),
                 new AprilTag(
                     3,
                     new Pose3d(
                         Units.inchesToMeters(610.0),
                         Units.inchesToMeters(176.0),
                         Units.inchesToMeters(19.25),
                         new Rotation3d(0.0, 0.0, Math.PI))),
                 new AprilTag(
                     4,
                     new Pose3d(
                         Units.inchesToMeters(635.375),
                         Units.inchesToMeters(272.0),
                         Units.inchesToMeters(27.25),
                         new Rotation3d(0.0, 0.0, Math.PI))),
                 new AprilTag(
                     5,
                     new Pose3d(
                         Units.inchesToMeters(14.25),
                         LoadingZone.doubleSubstationCenterY,
                         Units.inchesToMeters(27.38),
                         new Rotation3d())),
                 new AprilTag(
                     6,
                     new Pose3d(
                         Units.inchesToMeters(40.45),
                         Grids.nodeY[7],
                         Units.inchesToMeters(18.22),
                         new Rotation3d())),
                 new AprilTag(
                     7,
                     new Pose3d(
                         Units.inchesToMeters(40.45),
                         Grids.nodeY[4],
                         Units.inchesToMeters(18.22),
                         new Rotation3d())),
                 new AprilTag(
                     8,
                     new Pose3d(
                         Units.inchesToMeters(40.45),
                         Grids.nodeY[1],
                         Units.inchesToMeters(18.22),
                         new Rotation3d()))),
             fieldLength,
             fieldWidth)
         : new AprilTagFieldLayout(
             List.of(
                 new AprilTag(
                     1,
                     new Pose3d(
                         Units.inchesToMeters(610.77),
                         Grids.nodeY[1],
                         Units.inchesToMeters(18.22),
                         new Rotation3d(0.0, 0.0, Math.PI))),
                 new AprilTag(
                     2,
                     new Pose3d(
                         Units.inchesToMeters(610.77),
                         Grids.nodeY[4],
                         Units.inchesToMeters(18.22),
                         new Rotation3d(0.0, 0.0, Math.PI))),
                 new AprilTag(
                     3,
                     new Pose3d(
                         Units.inchesToMeters(610.77),
                         Grids.nodeY[7],
                         Units.inchesToMeters(18.22),
                         new Rotation3d(0.0, 0.0, Math.PI))),
                 new AprilTag(
                     4,
                     new Pose3d(
                         Units.inchesToMeters(636.96),
                         LoadingZone.doubleSubstationCenterY,
                         Units.inchesToMeters(27.38),
                         new Rotation3d(0.0, 0.0, Math.PI))),
                 new AprilTag(
                     5,
                     new Pose3d(
                         Units.inchesToMeters(14.25),
                         LoadingZone.doubleSubstationCenterY,
                         Units.inchesToMeters(27.38),
                         new Rotation3d())),
                 new AprilTag(
                     6,
                     new Pose3d(
                         Units.inchesToMeters(40.45),
                         Grids.nodeY[7],
                         Units.inchesToMeters(18.22),
                         new Rotation3d())),
                 new AprilTag(
                     7,
                     new Pose3d(
                         Units.inchesToMeters(40.45),
                         Grids.nodeY[4],
                         Units.inchesToMeters(18.22),
                         new Rotation3d())),
                 new AprilTag(
                     8,
                     new Pose3d(
                         Units.inchesToMeters(40.45),
                         Grids.nodeY[1],
                         Units.inchesToMeters(18.22),
                         new Rotation3d()))),
             fieldLength,
             fieldWidth);
}