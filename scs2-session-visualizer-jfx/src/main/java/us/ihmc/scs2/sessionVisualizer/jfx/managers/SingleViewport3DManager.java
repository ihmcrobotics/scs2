package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.scene.PerspectiveCamera;
import javafx.scene.layout.Pane;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.PerspectiveCameraController;

public interface SingleViewport3DManager extends Manager
{
   Pane getPane();

   void dispose();

   PerspectiveCameraController getCameraController();

   PerspectiveCamera getCamera();
}
