package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.concurrent.CountDownLatch;
import java.util.function.BooleanSupplier;

import com.sun.javafx.application.PlatformImpl;

import javafx.animation.AnimationTimer;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.util.Duration;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;

public class JavaFXMissingTools
{
   public static void addEquals(Translate translateToModify, Tuple2DReadOnly offset)
   {
      translateToModify.setX(translateToModify.getX() + offset.getX());
      translateToModify.setY(translateToModify.getY() + offset.getY());
   }

   public static void runLater(Class<?> caller, Runnable task)
   {
      Platform.runLater(task::run);
   }

   public static void runLaterIfNeeded(Class<?> caller, Runnable task)
   {
      if (Platform.isFxApplicationThread())
      {
         task.run();
      }
      else
      {
         try
         {
            runLater(caller, task);
         }
         catch (IllegalStateException e)
         {
            task.run();
         }
      }
   }

   public static void runNFramesLater(int numberOfFramesToWait, Runnable runnable)
   {
      new AnimationTimer()
      {
         int counter = 0;

         @Override
         public void handle(long now)
         {
            if (counter++ > numberOfFramesToWait)
            {
               runnable.run();
               stop();
            }
         }
      }.start();
   }

   public static void runAndWait(Class<?> caller, final Runnable runnable)
   {
      if (Platform.isFxApplicationThread())
      {
         try
         {
            runnable.run();
         }
         catch (Throwable t)
         {
            System.err.println("Exception in runnable");
            t.printStackTrace();
         }
      }
      else
      {
         final CountDownLatch doneLatch = new CountDownLatch(1);

         runLater(caller, () ->
         {
            try
            {
               runnable.run();
            }
            finally
            {
               doneLatch.countDown();
            }
         });

         try
         {
            doneLatch.await();
         }
         catch (InterruptedException ex)
         {
            ex.printStackTrace();
         }
      }
   }

   public static void runLaterWhen(Class<?> caller, BooleanSupplier condition, Runnable runnable)
   {
      new ObservedAnimationTimer(caller.getSimpleName())
      {
         @Override
         public void handleImpl(long now)
         {
            if (condition.getAsBoolean())
            {
               try
               {
                  runnable.run();
               }
               catch (Exception e)
               {
                  e.printStackTrace();
               }
               finally
               {
                  stop();
               }
            }
         }
      }.start();
   }

   public static void setAnchorConstraints(Node child, double allSides)
   {
      setAnchorConstraints(child, allSides, allSides, allSides, allSides);
   }

   public static void setAnchorConstraints(Node child, double top, double right, double bottom, double left)
   {
      AnchorPane.setTopAnchor(child, top);
      AnchorPane.setRightAnchor(child, right);
      AnchorPane.setBottomAnchor(child, bottom);
      AnchorPane.setLeftAnchor(child, left);
   }

   public static Application splashScreen(Image image)
   {
      Application splashScreenApplication = new Application()
      {
         private Stage primaryStage;

         @Override
         public void start(Stage primaryStage) throws Exception
         {
            this.primaryStage = primaryStage;
            primaryStage.initStyle(StageStyle.TRANSPARENT);
            ImageView imageView = new ImageView(image);
            imageView.setOpacity(0.0);
            Scene scene = new Scene(new Pane(imageView));
            scene.setFill(Color.TRANSPARENT);
            primaryStage.setScene(scene);
            SessionVisualizerIOTools.addSCSIconToWindow(primaryStage);
            primaryStage.show();

            Timeline timeline = new Timeline(new KeyFrame(Duration.seconds(1.0), new KeyValue(imageView.opacityProperty(), 1.0)));
            timeline.playFromStart();
         }

         @Override
         public void stop() throws Exception
         {
            super.stop();
            if (primaryStage == null)
               return;
            primaryStage.close();
            primaryStage = null;
         }
      };
      runApplication(splashScreenApplication);
      return splashScreenApplication;
   }

   public static void runApplication(Application application)
   {
      runApplication(application, null);
   }

   public static void runApplication(Application application, Runnable initialize)
   {
      Runnable runnable = () ->
      {
         try
         {
            application.start(new Stage());
            if (initialize != null)
               initialize.run();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      };

      PlatformImpl.startup(() ->
      {
         runLater(application.getClass(), runnable);
      });
      PlatformImpl.setImplicitExit(false);
   }
}
