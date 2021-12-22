package us.ihmc.scs2.sessionVisualizer.jfx.session;

import javafx.stage.Stage;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;

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

   default void bringUp()
   {
      getStage().setIconified(false);
      getStage().setMaximized(false);
      getStage().setFullScreen(false);
      getStage().centerOnScreen();
      getStage().toFront();
      getStage().show();
   }
}
