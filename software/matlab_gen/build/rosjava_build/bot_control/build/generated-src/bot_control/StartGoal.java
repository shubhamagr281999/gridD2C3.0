package bot_control;

public interface StartGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "bot_control/StartGoal";
  static final java.lang.String _DEFINITION = "int16[] start_x\nint16[] start_y\nint16[] start_d\nint16[] goal_x\nint16[] goal_y\nint16[] goal_d\nint16[] bot_num";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short[] getStartX();
  void setStartX(short[] value);
  short[] getStartY();
  void setStartY(short[] value);
  short[] getStartD();
  void setStartD(short[] value);
  short[] getGoalX();
  void setGoalX(short[] value);
  short[] getGoalY();
  void setGoalY(short[] value);
  short[] getGoalD();
  void setGoalD(short[] value);
  short[] getBotNum();
  void setBotNum(short[] value);
}
