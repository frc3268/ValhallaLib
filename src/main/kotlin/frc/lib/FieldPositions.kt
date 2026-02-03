package frc.lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d

/** extension function to convert Int to Rotation3d from degrees */
fun Int.rotation3dFromDeg(): Rotation3d = Rotation3d(0.0, 0.0, Math.toRadians(this.toDouble()))

// Field Location has two poses, one for red alliance and one for blue alliance
data class FieldLocation(val red: Pose3d, val blue: Pose3d)

// +Z is up into the air from the carpet, +X is horizontal to the right (Based off the image below), +Y runs from the Field Border towards the REEFS.
// for the z-rotation, 180° faces the arena center, 0° faces the alliance station side. 90° faces the non-scoring table side.
// Distances are measured to the center of the tag.
/** Object used to store position of field elements
 * April Tag info can be found on page 11 of https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
 *
 * ^ use this to get more info on each point provided here. Raw coordinate data provided in fieldpositions.csv */
object FieldPositions {

    fun closest(startingPose: Pose2d, locations: List<FieldLocation>): FieldLocation {
        val red = locations.minByOrNull { startingPose.translation.getDistance(it.red.toPose2d().translation) }!!
        val blue = locations.minByOrNull { startingPose.translation.getDistance(it.blue.toPose2d().translation) }!!
        return FieldLocation(red.red, blue.blue)
    }

    /** Hub facing the INSIDE of the arena */
    val hubInside = listOf(
        /** coordinates for AprilTag 5 and AprilTag 18 */
        FieldLocation(
            Pose3d(469.11, 135.09, 44.25, 270.rotation3dFromDeg()),
            Pose3d(182.11, 135.09, 44.25, 270.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 4 and AprilTag 19 */
        FieldLocation(
            Pose3d(445.35, 158.84, 44.24, 180.rotation3dFromDeg()),
            Pose3d(205.87, 144.84, 44.25, 0.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 3 and AprilTag 20 */
        FieldLocation(
            Pose3d(445.35, 172.84, 44.25, 180.rotation3dFromDeg()),
            Pose3d(205.87, 158.84, 44.25, 0.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 2 and AprilTag 21 */
        FieldLocation(
            Pose3d(468.11, 182.60, 44.25, 90.rotation3dFromDeg()),
            Pose3d(182.11, 182.60, 44.25, 90.rotation3dFromDeg())
        ),
    )

    /** Hub facing the OUTSIDE of the arena */
    val hubOutside = listOf(
        /** coordinates for AprilTag 8 and AprilTag 27 */
        FieldLocation(
            Pose3d(483.11, 135.09, 44.25, 270.rotation3dFromDeg()),
            Pose3d(168.11, 135.09, 44.25, 270.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 9 and AprilTag 26 */
        FieldLocation(
            Pose3d(492.88, 144.84, 44.25, 0.rotation3dFromDeg()),
            Pose3d(158.34, 158.84, 442.25, 180.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 10 and AprilTag 25 */
        FieldLocation(
            Pose3d(492.88, 158.84, 44.25, 0.rotation3dFromDeg()),
            Pose3d(158.34, 172.84, 44.25, 180.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 11 and AprilTag 24 */
        FieldLocation(
            Pose3d(483.11, 182.60, 44.25, 0.rotation3dFromDeg()),
            Pose3d(168.11, 182.60, 44.25, 90.rotation3dFromDeg())
        )
    )

    /** Trench facing the INSIDE of the arena */
    val trenchInside = listOf(
        /** coordinates for AprilTag 1 and AprilTag 22 */
        FieldLocation(
            Pose3d(467.64, 292.31, 35.0, 180.rotation3dFromDeg()),
            Pose3d(183.59, 292.31, 35.0, 0.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 6 and AprilTag 17 */
        FieldLocation(
            Pose3d(467.64, 25.37, 35.00, 180.rotation3dFromDeg()),
            Pose3d(183.59, 25.37, 35.00, 0.rotation3dFromDeg())
        )
    )

    /** Trench facing the OUTSIDE of the arena */
    val trenchOutside = listOf(
        /** coordinates for AprilTag 12 and AprilTag 17 */
        FieldLocation(
            Pose3d(470.59, 292.31, 35.00, 0.rotation3dFromDeg()),
            Pose3d(183.59, 25.37, 35.00, 0.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 7 and AprilTag 28 */
        FieldLocation(
            Pose3d(470.59, 25.37, 35.00, 0.rotation3dFromDeg()),
            Pose3d(180.64, 25.37, 35.00, 180.rotation3dFromDeg())
        )
    )

    /** Tower */
    val tower = listOf(
        /** coordinates for AprilTag 16 and AprilTag 31 */
        FieldLocation(
            Pose3d(650.90, 153.22, 21.75, 180.rotation3dFromDeg()),
            Pose3d(0.32, 147.47, 21.75, 0.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 15 and AprilTag 32 */
        FieldLocation(
            Pose3d(650.90, 170.22, 21.75, 180.rotation3dFromDeg()),
            Pose3d(0.32, 164.47, 21.75, 0.rotation3dFromDeg())
        )
    )

    /** Outpost */
    val outpost = listOf(
        /** coordinates for AprilTag 13 and AprilTag 30 */
        FieldLocation(
            Pose3d(650.92, 291.47, 21.75, 180.rotation3dFromDeg()),
            Pose3d(0.30, 43.22, 21.75, 0.rotation3dFromDeg())
        ),
        /** coordinates for AprilTag 14 and AprilTag 29 */
        FieldLocation(
            Pose3d(650.92, 274.47, 21.75, 180.rotation3dFromDeg()),
            Pose3d(0.30, 26.22, 21.75, 0.rotation3dFromDeg())
        )
    )

}
