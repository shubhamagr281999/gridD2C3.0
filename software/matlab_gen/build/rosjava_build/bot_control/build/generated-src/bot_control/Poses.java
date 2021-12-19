package bot_control;

public interface Poses extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "bot_control/Poses";
  static final java.lang.String _DEFINITION = "geometry_msgs/Point[] posei";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.util.List<geometry_msgs.Point> getPosei();
  void setPosei(java.util.List<geometry_msgs.Point> value);
}
