package us.ihmc.scs2.sessionVisualizer.jfx.session;

import javafx.stage.Window;

public class OpenAddLogRequest
{
   private final Window source;

   public OpenAddLogRequest(Window source)
   {
      this.source = source;
   }

   public Window getSource()
   {
      return source;
   }
}
