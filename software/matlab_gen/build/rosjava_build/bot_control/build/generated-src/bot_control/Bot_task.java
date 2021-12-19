package bot_control;

public interface Bot_task extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "bot_control/Bot_task";
  static final java.lang.String _DEFINITION = "bool[] task\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  boolean[] getTask();
  void setTask(boolean[] value);
}
