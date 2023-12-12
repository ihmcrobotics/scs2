package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import java.util.function.Consumer;

import com.sun.javafx.application.PlatformImpl;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;

public class ApplicationRunner
{
   public static void runApplication(Consumer<Stage> primaryStageConsumer)
   {
      runApplication(new Application()
      {
         @Override
         public void start(Stage primaryStage) throws Exception
         {
            primaryStageConsumer.accept(primaryStage);
         }
      });
   }

   public static Runnable runApplication(Application launcher)
   {
      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               launcher.start(new Stage());
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      };

      PlatformImpl.startup(() -> Platform.runLater(runnable));

      return runnable;
   }
}
