package frc.lib

import edu.wpi.first.math.geometry.Pose2d
import java.lang.Math.pow
import java.util.*


data class Point(val loc: Pair<Int, Int>, val from: Point?)

fun get_f(loc: Pair<Int, Int>, dest: Pair<Int, Int>) = pow(
    pow(loc.first.toDouble() - dest.first.toDouble(), 2.0) + pow(
        loc.second.toDouble() - dest.second.toDouble(),
        2.0
    ), 0.5
)

fun construct_path(closed: MutableList<Point>): MutableList<Pose2d> {
    val first = closed.last()
    val list: MutableList<Pose2d> = mutableListOf()
    var next = first.from
    while (next != null) {
        list.add(0, Pose2d(next.loc.first * 0.3 + 0.15, next.loc.second * 0.3 + 0.15, 0.0.rotation2dFromDeg()))
        next = next.from
    }
    return list
}

val directions = listOf(
    Pair(-1, 0), // left
    Pair(1, 0),  // right
    Pair(0, -1), // up
    Pair(0, 1),  // down
    Pair(-1, -1), // top-left diagonal
    Pair(1, -1),  // top-right diagonal
    Pair(-1, 1),  // bottom-left diagonal
    Pair(1, 1)    // bottom-right diagonal
)

//"field_size":{"x":17.548,"y":8.052},"nodeSizeMeters":0.3,


fun a_star(start: Pair<Int, Int>, end: Pair<Int, Int>, grid: Array<Array<Boolean>>): MutableList<Point> {
    val allPoints: MutableMap<Pair<Int, Int>, Double> = mutableMapOf(
        start to (get_f(start, end))
    )
    val compareByLength: Comparator<Point> = compareBy { allPoints[it.loc] }
    val openqueue: PriorityQueue<Point> = PriorityQueue<Point>(compareByLength)
    val closed: MutableList<Point> = mutableListOf(Point(start, null))
    var rootNow = closed.last()
    while (rootNow.loc != end) {
        println(end.first)
        println(rootNow.loc.first)
        for (direction in directions) {
            if (rootNow.loc.second + direction.second in 0..grid.size - 1) {
                if (rootNow.loc.first + direction.first in 0..grid[rootNow.loc.second + direction.second].size - 1 && !grid[rootNow.loc.second + direction.second][rootNow.loc.first + direction.first] && !closed.map { it.loc }
                        .contains(Pair(rootNow.loc.first + direction.first, rootNow.loc.second + direction.second))) {
                    allPoints.set(
                        Pair(rootNow.loc.first + direction.first, rootNow.loc.second + direction.second),
                        (get_f(Pair(rootNow.loc.first + direction.first, rootNow.loc.second + direction.second), end))
                    )
                    openqueue.add(
                        Point(
                            Pair(
                                rootNow.loc.first + direction.first,
                                rootNow.loc.second + direction.second
                            ), rootNow
                        )
                    )
                }
            }
        }
        closed.add(openqueue.peek())
        openqueue.remove()
        rootNow = closed.last()
    }
    return closed
}

fun smoothPath(poses: MutableList<Pose2d>): MutableList<Pose2d> {
    val movingA: MutableList<Pose2d> = mutableListOf()
    var count = 0
    for (i in 0..poses.size - 5) {
        if (count % 5 == 0) {
            movingA.add(
                Pose2d(
                    (poses[i].x + poses[i + 1].x + poses[i + 2].x + poses[i + 3].x + poses[i + 4].x) / 5,
                    (poses[i].y + poses[i + 1].y + poses[i + 2].y + poses[i + 3].y + poses[i + 4].y) / 5,
                    0.0.rotation2dFromDeg()
                )
            )
        }
        count++
    }
    return movingA
}
