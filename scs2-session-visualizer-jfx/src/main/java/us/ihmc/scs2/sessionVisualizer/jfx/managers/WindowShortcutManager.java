package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.event.EventHandler;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.stage.Window;
import us.ihmc.messager.Messager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;

public class WindowShortcutManager
{
   private final Window owner;
   private final EventHandler<KeyEvent> currentIndexSteppingListener;

   public WindowShortcutManager(Window owner, Messager messager, SessionVisualizerTopics topics)
   {
      this.owner = owner;
      currentIndexSteppingListener = keyEvent ->
      {
         if (!keyEvent.isAltDown())
            return;

         int step;

         if (keyEvent.getCode() == KeyCode.LEFT || keyEvent.getCode() == KeyCode.KP_LEFT)
            step = -1;
         else if (keyEvent.getCode() == KeyCode.RIGHT || keyEvent.getCode() == KeyCode.KP_RIGHT)
            step = 1;
         else
            return;

         if (keyEvent.isShiftDown())
            step *= 10;

         messager.submitMessage(topics.getYoBufferIncrementCurrentIndexRequest(), step);
         keyEvent.consume();
      };

      owner.setOnCloseRequest(e -> stop());
   }

   public void start()
   {
      owner.addEventHandler(KeyEvent.KEY_PRESSED, currentIndexSteppingListener);
   }

   public void stop()
   {
      owner.removeEventHandler(KeyEvent.KEY_PRESSED, currentIndexSteppingListener);
   }
}
