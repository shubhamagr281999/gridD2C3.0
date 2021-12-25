package bot_control;

public interface Bot_status extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "bot_control/Bot_status";
  static final java.lang.String _DEFINITION = "int16[] status\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short[] getStatus();
  void setStatus(short[] value);
}
