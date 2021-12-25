package bot_control;

public interface CompletePlan extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "bot_control/CompletePlan";
  static final java.lang.String _DEFINITION = "bot_control/PathArray[] agent";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.util.List<bot_control.PathArray> getAgent();
  void setAgent(java.util.List<bot_control.PathArray> value);
}
