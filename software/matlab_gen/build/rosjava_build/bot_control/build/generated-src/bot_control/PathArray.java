package bot_control;

public interface PathArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "bot_control/PathArray";
  static final java.lang.String _DEFINITION = "geometry_msgs/Point[] statei\nint16 bot_num";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.util.List<geometry_msgs.Point> getStatei();
  void setStatei(java.util.List<geometry_msgs.Point> value);
  short getBotNum();
  void setBotNum(short value);
}
