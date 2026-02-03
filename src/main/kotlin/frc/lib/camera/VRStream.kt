package frc.lib.camera

import edu.wpi.first.wpilibj.DriverStation
import org.opencv.videoio.VideoCapture

class VRStream {

    private var loaded: Boolean = false;

    var fps: UInt = 30u;

    init {
        // TODO: Load Webcam
        val video = VideoCapture(0)
        if (video.isOpened) {
            // Do Something I guess
            this.loaded = true;
        } else {
            DriverStation.reportWarning("VRStream: Failed to Load", false)
        }

    }
}