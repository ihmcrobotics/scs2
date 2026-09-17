package us.ihmc.scs2.sessionVisualizer.jfx.session;

import javafx.stage.Stage;
import javafx.stage.Window;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public interface SessionControlsController
{
   void initialize(SessionVisualizerToolkit toolkit);

   /**
    * Bind these controls to a session that is already running in the toolkit (CLI {@code -l}, or a
    * drop that started the session without opening this window). No-op when this controller does not
    * handle that session type, or when it is already bound to the same instance.
    */
   default void bindRunningSession(Session session)
   {
   }

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
