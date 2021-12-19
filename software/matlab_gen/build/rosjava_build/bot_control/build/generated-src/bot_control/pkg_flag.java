package bot_control;

public interface pkg_flag extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "bot_control/pkg_flag";
  static final java.lang.String _DEFINITION = "int16 bot_num\nint16 LS";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short getBotNum();
  void setBotNum(short value);
  short getLS();
  void setLS(short value);
}
