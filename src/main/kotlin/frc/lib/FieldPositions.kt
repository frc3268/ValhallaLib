package frc.lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d

import edu.wpi.first.math.geometry.Rotation3d

fun Int.rotation3dFromDeg() = Rotation3d.fromDegrees(this.toDouble())

//for clarity, blue is the pose of the field element on the blue side, and red is the same but on the red side
data class FieldLocation(val red: Pose3d, val blue: Pose3d)
data class Obstacle(val location: Pose2d, val radiusMeters: Double)

// TODO: IMPLEMENT! Size is in inches.
// +Z is up into the air from the carpet, +X is horizontal to the right (Based off the image below), +Y runs from the Field Border towards the REEFS.
// For the Z-Rotation (I presume?) 0° faces the red alliance station, 90° faces the non- scoring table side, and 180° faces the blue alliance station.
// For the X-Rotation, 0 is perpendicular to the Z plane, and 90 degrees is facing the carpet. Distances are measured to the center of the tag.
// Distances are measured to the center of the tag.
/** Object used to store position of field elements */
/** April Tag info can be found on page 11 of https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf */
object FieldPositions {

    fun closest(startingPose: Pose2d, locations: List<FieldLocation>): FieldLocation {
        val red = locations.minByOrNull{ startingPose.translation.getDistance(it.red.translation)}!!
        val blue = locations.minByOrNull{ startingPose.translation.getDistance(it.blue.translation)}!!
        return FieldLocation(red.red, blue.blue)
    }

    /** Hub facing the INSIDE of the arena */
    val hubInside = listOf(
        /** coordinates for AprilTag 5 and AprilTag 18 */
        FieldLocation(Pose3d(469.11, 135.09, 44.25, 270.rotation3dFromDeg()),Pose3d(182.11, 135.09, 44.25, 270.rotation3dFromDeg())),
        /** coordinates for AprilTag 4 and AprilTag 19 */
        FieldLocation(Pose3d(445.35,158.84,44.24, 180.rotation3dFromDeg()), Pose3d(205.87,144.84,44.25, 0.rotation3dFromDeg())),
        /** coordinates for AprilTag 3 and AprilTag 20 */
        FieldLocation(Pose3d(445.35,172.84,44.25,180.rotation3dFromDeg()), Pose3d(205.87,158.84,44.25,0.rotation3dFromDeg())),
        /** coordinates for AprilTag 2 and AprilTag 21 */
        FieldLocation(Pose3d(468.11,182.60,44.25,90.rotation3dFromDeg()),Pose3d(182.11,182.60,44.25,90.rotation3dFromDeg())),
        )

    /** Hub facing the OUTSIDE of the arena */
    val hubOutside = listOf(
        /** coordinates for AprilTag 8 and AprilTag 27 */
        FieldLocation(Pose3d(483.11,135.09,44.25,270.rotation3dFromDeg()), Pose3d(168.11,135.09,44.25,270.rotation3dFromDeg())),
        /** coordinates for AprilTag 9 and AprilTag 26 */
        FieldLocation(Pose3d(492.88,144.84,44.25,0.rotation3dFromDeg()), Pose3d(158.34,158.84,442.25,180.rotation3dFromDeg())),
        /** coordinates for AprilTag 10 and AprilTag 25 */
        FieldLocation(Pose3d(492.88,158.84,44.25,0.rotation3dFromDeg()),Pose3d(158.34,172.84,44.25,180.rotation3dFromDeg())),
        /** coordinates for AprilTag 11 and AprilTag 24 */
        FieldLocation(Pose3d(483.11,182.60,44.25,0.rotation3dFromDeg()),Pose3d(168.11,182.60,44.25,90.rotation3dFromDeg()))
    )
//    /** Trench facing the INSIDE of the arena */
//    val trenchInside = listOf(
//        /** coordinates for AprilTag 1 and AprilTag 22 */
//        FieldLocation(Pose3d(467.64,292.31,35,180.rotation3dFromDeg())),Pose3d(183.59,292.31,35,0.rotation3dFromDeg())),
//    /** coordinates for AprilTag 11 and AprilTag 24 */
//    FieldLocation(Pose3d()),Pose3d())
//
//    )
    )
}
