package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// I didn't make this class at all, I just copied it from github:
// https://github.com/frc5687/2018-robot/blob/master/src/main/java/org/frc5687/powerup/robot/utils/LidarProxy.java
public class LidarProxy {

    private double lastReadDistance;
    private LidarListener _listener;
    private Thread _thread;
    private boolean _initializedProperly;

    public LidarProxy(SerialPort.Port port) {
        setup(port);
    }

    private void setup(SerialPort.Port port) {
        try {
            _listener = new LidarListener(this, port);
            _thread = new Thread(_listener);
            _thread.start();
            _initializedProperly = true;
        } catch (Exception e) {
            _initializedProperly = false;
            DriverStation.reportError(
                "LidarProxy could not intialize properly. " + e.getStackTrace().toString(),
                false
            );
        }
        SmartDashboard.putBoolean("Lidar/initializedProperly", _initializedProperly);
    }

    public double get() {
        return lastReadDistance;
    }

    protected class LidarListener implements Runnable {

        private SerialPort _port;
        private LidarProxy _proxy;

        protected LidarListener(LidarProxy proxy, SerialPort.Port port) {
            _proxy = proxy;
            _port = new SerialPort(115200, port);
            _port.setReadBufferSize(9);
        }

        public void run() {
            while (true) {
                try {
                    SmartDashboard.putNumber("Lidar/_port.getBytesReceived()", _port.getBytesReceived());
                    byte[] read = _port.read(9);
                    SmartDashboard.putNumber("Lidar/readLength", read.length);
                    SmartDashboard.putNumber("Lidar/bytes/3", new Integer(read[2] & 0xFF));
                    _proxy.lastReadDistance = read[2] & 0xFF;
                } catch (Exception e) {
                    DriverStation.reportError("LidarListener exception: " + e.toString(), false);
                }
            }
        }
    }
}
// alex is weird
