package us.ihmc.scs2.sessionVisualizer.jfx.session;

import javafx.stage.Window;

public class OpenSessionControlsRequest
{
   public enum SessionType
   {
      LOG, REMOTE
   };

   private final Window source;
   private final SessionType sessionType;

   public OpenSessionControlsRequest(Window source, SessionType sessionType)
   {
      this.source = source;
      this.sessionType = sessionType;
   }

   public Window getSource()
   {
      return source;
   }

   public SessionType getSessionType()
   {
      return sessionType;
   }
}
