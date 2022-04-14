package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.scene.PerspectiveCamera;
import javafx.scene.layout.Pane;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;

public interface SingleViewport3DManager
{

   Pane getPane();

   void dispose();

   FocusBasedCameraMouseEventHandler getCameraController();

   PerspectiveCamera getCamera();

}
