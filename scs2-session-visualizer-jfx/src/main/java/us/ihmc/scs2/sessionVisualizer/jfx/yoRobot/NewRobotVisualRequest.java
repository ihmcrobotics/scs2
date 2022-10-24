package us.ihmc.scs2.sessionVisualizer.jfx.yoRobot;

import javafx.scene.shape.DrawMode;

public class NewRobotVisualRequest
{
   public static final String ALL_ROBOTS = "AllOfTheRobots!";

   private String robotName = null;
   private DrawMode requestedDrawMode = null;
   private Boolean requestedVisible = null;

   public static NewRobotVisualRequest wireframeMode(String robotName, boolean enable)
   {
      NewRobotVisualRequest request = new NewRobotVisualRequest();
      request.setRobotName(robotName);
      request.setRequestedDrawMode(enable ? DrawMode.LINE : DrawMode.FILL);
      return request;
   }

   public static NewRobotVisualRequest visible(String robotName, boolean visible)
   {
      NewRobotVisualRequest request = new NewRobotVisualRequest();
      request.setRobotName(robotName);
      request.setRequestedVisible(Boolean.valueOf(visible));
      return request;
   }

   public NewRobotVisualRequest()
   {
   }

   public void setRobotName(String robotName)
   {
      this.robotName = robotName;
   }

   public void setRequestedDrawMode(DrawMode requestedDrawMode)
   {
      this.requestedDrawMode = requestedDrawMode;
   }

   public void setRequestedVisible(Boolean requestedVisible)
   {
      this.requestedVisible = requestedVisible;
   }

   public String getRobotName()
   {
      return robotName;
   }

   public DrawMode getRequestedDrawMode()
   {
      return requestedDrawMode;
   }

   public Boolean getRequestedVisible()
   {
      return requestedVisible;
   }
}
