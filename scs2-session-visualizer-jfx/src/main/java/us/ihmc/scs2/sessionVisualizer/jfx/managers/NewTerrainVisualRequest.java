package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.scene.shape.DrawMode;

public class NewTerrainVisualRequest
{
   private DrawMode requestedDrawMode = null;
   private Boolean requestedVisible = null;

   public static NewTerrainVisualRequest wireframeMode(boolean enable)
   {
      NewTerrainVisualRequest request = new NewTerrainVisualRequest();
      request.setRequestedDrawMode(enable ? DrawMode.LINE : DrawMode.FILL);
      return request;
   }

   public static NewTerrainVisualRequest visible(boolean visible)
   {
      NewTerrainVisualRequest request = new NewTerrainVisualRequest();
      request.setRequestedVisible(Boolean.valueOf(visible));
      return request;
   }

   public void setRequestedDrawMode(DrawMode requestedDrawMode)
   {
      this.requestedDrawMode = requestedDrawMode;
   }

   public void setRequestedVisible(Boolean requestedVisible)
   {
      this.requestedVisible = requestedVisible;
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
