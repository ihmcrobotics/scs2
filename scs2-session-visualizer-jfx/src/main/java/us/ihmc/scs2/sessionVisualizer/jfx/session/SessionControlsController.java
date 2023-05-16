package us.ihmc.scs2.sessionVisualizer.jfx.session;

import javafx.stage.Stage;
import javafx.stage.Window;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public interface SessionControlsController
{
   void initialize(SessionVisualizerToolkit toolkit);

   void notifySessionLoaded();

   void unloadSession();

   default SessionInfoController getSessionInfoController()
   {
      return null;
   }

   void shutdown();

   Stage getStage();

   default void bringUp(Window owner)
   {
      getStage().setIconified(false);
      getStage().setMaximized(false);
      getStage().setFullScreen(false);
      getStage().centerOnScreen();
      getStage().toFront();
      getStage().show();
      JavaFXMissingTools.centerWindowInOwner(getStage(), owner);
   }
}
