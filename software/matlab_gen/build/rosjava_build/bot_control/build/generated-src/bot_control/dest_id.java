package bot_control;

public interface dest_id extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "bot_control/dest_id";
  static final java.lang.String _DEFINITION = "int16 bot_num\nint16 LS\nint16 dest_id";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short getBotNum();
  void setBotNum(short value);
  short getLS();
  void setLS(short value);
  short getDestId();
  void setDestId(short value);
}
