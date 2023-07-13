package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.scene.PerspectiveCamera;
import javafx.scene.layout.Pane;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.FocusBasedCameraMouseEventHandler;

public interface SingleViewport3DManager
{

   Pane getPane();

   void dispose();

   FocusBasedCameraMouseEventHandler getCameraController();

   PerspectiveCamera getCamera();

}
