package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import com.sun.javafx.application.PlatformImpl;
import javafx.animation.AnimationTimer;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Dialog;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.Shape3D;
import javafx.scene.transform.Affine;
import javafx.scene.transform.NonInvertibleTransformException;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;

import java.util.concurrent.Callable;
import java.util.concurrent.CountDownLatch;
import java.util.function.BooleanSupplier;

public class JavaFXMissingTools
{
   public static void zero(Translate translate)
   {
      translate.setX(0.0);
      translate.setY(0.0);
      translate.setZ(0.0);
   }

   public static void setTranslate(Translate translateToPack, double x, double y, double z)
   {
      translateToPack.setX(x);
      translateToPack.setY(y);
      translateToPack.setZ(z);
   }

   public static void addEquals(Translate translateToModify, Tuple2DReadOnly offset)
   {
      translateToModify.setX(translateToModify.getX() + offset.getX());
      translateToModify.setY(translateToModify.getY() + offset.getY());
   }

   public static void addEquals(Translate translateToModify, Tuple3DReadOnly offset)
   {
      addEquals(translateToModify, offset.getX(), offset.getY(), offset.getZ());
   }

   public static void addEquals(Translate translateToModify, double dx, double dy, double dz)
   {
      translateToModify.setX(translateToModify.getX() + dx);
      translateToModify.setY(translateToModify.getY() + dy);
      translateToModify.setZ(translateToModify.getZ() + dz);
   }

   public static void subEquals(Translate translateToModify, Tuple3DReadOnly offset)
   {
      translateToModify.setX(translateToModify.getX() - offset.getX());
      translateToModify.setY(translateToModify.getY() - offset.getY());
      translateToModify.setZ(translateToModify.getZ() - offset.getZ());
   }

   public static void applyTranform(Transform transform, Vector3DBasics vectorToTransform)
   {
      javafx.geometry.Point3D temporaryVector = transform.deltaTransform(vectorToTransform.getX(), vectorToTransform.getY(), vectorToTransform.getZ());
      vectorToTransform.set(temporaryVector.getX(), temporaryVector.getY(), temporaryVector.getZ());
   }

   public static void applyTranform(Transform transform, Point3DBasics pointToTransform)
   {
      javafx.geometry.Point3D temporaryVector = transform.transform(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ());
      pointToTransform.set(temporaryVector.getX(), temporaryVector.getY(), temporaryVector.getZ());
   }

   public static void applyInvertTranform(Transform transform, Vector3DBasics vectorToTransform)
   {
      javafx.geometry.Point3D temporaryVector = new javafx.geometry.Point3D(vectorToTransform.getX(), vectorToTransform.getY(), vectorToTransform.getZ());
      try
      {
         transform.inverseDeltaTransform(temporaryVector);
      }
      catch (NonInvertibleTransformException e)
      {
         e.printStackTrace();
      }
      vectorToTransform.set(temporaryVector.getX(), temporaryVector.getY(), temporaryVector.getZ());
   }

   public static void convertAxisAngleToRotate(AxisAngleReadOnly axisAngle, Rotate rotateToPack)
   {
      rotateToPack.setAngle(axisAngle.getAngle());
      rotateToPack.setPivotX(0.0);
      rotateToPack.setPivotY(0.0);
      rotateToPack.setPivotZ(0.0);
      rotateToPack.setAxis(new javafx.geometry.Point3D(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ()));
   }

   public static Affine createAffineFromOrientation3DAndTuple(Orientation3DReadOnly orientation3D, Tuple3DReadOnly translation)
   {
      return createRigidBodyTransformToAffine(new RigidBodyTransform(orientation3D, translation));
   }

   public static Affine createRigidBodyTransformToAffine(RigidBodyTransform rigidBodyTransform)
   {
      Affine ret = new Affine();
      convertRigidBodyTransformToAffine(rigidBodyTransform, ret);
      return ret;
   }

   public static void convertRigidBodyTransformToAffine(RigidBodyTransform rigidBodyTransform, Affine affineToPack)
   {
      affineToPack.setMxx(rigidBodyTransform.getM00());
      affineToPack.setMxy(rigidBodyTransform.getM01());
      affineToPack.setMxz(rigidBodyTransform.getM02());
      affineToPack.setMyx(rigidBodyTransform.getM10());
      affineToPack.setMyy(rigidBodyTransform.getM11());
      affineToPack.setMyz(rigidBodyTransform.getM12());
      affineToPack.setMzx(rigidBodyTransform.getM20());
      affineToPack.setMzy(rigidBodyTransform.getM21());
      affineToPack.setMzz(rigidBodyTransform.getM22());

      affineToPack.setTx(rigidBodyTransform.getM03());
      affineToPack.setTy(rigidBodyTransform.getM13());
      affineToPack.setTz(rigidBodyTransform.getM23());
   }

   public static void convertRotationMatrixToAffine(RotationMatrixReadOnly rotation, Affine affineToModify)
   {
      affineToModify.setMxx(rotation.getM00());
      affineToModify.setMxy(rotation.getM01());
      affineToModify.setMxz(rotation.getM02());
      affineToModify.setMyx(rotation.getM10());
      affineToModify.setMyy(rotation.getM11());
      affineToModify.setMyz(rotation.getM12());
      affineToModify.setMzx(rotation.getM20());
      affineToModify.setMzy(rotation.getM21());
      affineToModify.setMzz(rotation.getM22());
   }

   public static void convertEuclidAffineToJavaFXAffine(AffineTransform euclidAffine, Affine javaFxAffineToPack)
   {
      javaFxAffineToPack.setMxx(euclidAffine.getM00());
      javaFxAffineToPack.setMxy(euclidAffine.getM01());
      javaFxAffineToPack.setMxz(euclidAffine.getM02());
      javaFxAffineToPack.setMyx(euclidAffine.getM10());
      javaFxAffineToPack.setMyy(euclidAffine.getM11());
      javaFxAffineToPack.setMyz(euclidAffine.getM12());
      javaFxAffineToPack.setMzx(euclidAffine.getM20());
      javaFxAffineToPack.setMzy(euclidAffine.getM21());
      javaFxAffineToPack.setMzz(euclidAffine.getM22());

      javaFxAffineToPack.setTx(euclidAffine.getM03());
      javaFxAffineToPack.setTy(euclidAffine.getM13());
      javaFxAffineToPack.setTz(euclidAffine.getM23());
   }

   public static void runLater(Class<?> caller, Runnable task)
   {
      Platform.runLater(task::run);
   }

   public static void runLaterIfNeeded(Class<?> caller, Runnable runnable)
   {
      if (Platform.isFxApplicationThread())
      {
         tryRun(runnable);
      }
      else
      {
         try
         {
            runLater(caller, runnable);
         }
         catch (IllegalStateException e)
         {
            tryRun(runnable);
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
               tryRun(runnable);
               stop();
            }
         }
      }.start();
   }

   public static void runAndWait(Class<?> caller, final Runnable runnable)
   {
      if (Platform.isFxApplicationThread())
      {
         tryRun(runnable);
      }
      else
      {
         final CountDownLatch doneLatch = new CountDownLatch(1);

         runLater(caller, () ->
         {
            try
            {
               tryRun(runnable);
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

   public static void tryRun(final Runnable runnable)
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

   public static <R> R runAndWait(Class<?> caller, final Callable<R> callable)
   {
      if (Platform.isFxApplicationThread())
      {
         try
         {
            return callable.call();
         }
         catch (Throwable t)
         {
            LogTools.error("Exception in callable");
            t.printStackTrace();
            return null;
         }
      }
      else
      {
         final CountDownLatch doneLatch = new CountDownLatch(1);
         final MutableObject<R> result = new MutableObject<>();

         runLater(caller, () ->
         {
            try
            {
               result.setValue(callable.call());
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
            finally
            {
               doneLatch.countDown();
            }
         });

         try
         {
            doneLatch.await();
            return result.getValue();
         }
         catch (InterruptedException ex)
         {
            ex.printStackTrace();
            return null;
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
                  tryRun(runnable);
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

   public static void centerDialogInOwner(Dialog<?> dialog)
   {
      Window owner = dialog.getOwner();

      if (owner == null)
      {
         LogTools.error("This dialog has no owner set: {}", dialog);
         return;
      }

      dialog.setX(owner.getX() + 0.5 * (owner.getWidth() - dialog.getWidth()));
      dialog.setY(owner.getY() + 0.5 * (owner.getHeight() - dialog.getHeight()));

      if (!dialog.isShowing())
      {
         // TODO Seems that on Ubuntu the changes done to the window position/size are not processed properly until the window is showing.
         // This may be related to the bug reported when using GTK3: https://github.com/javafxports/openjdk-jfx/pull/446, might be fixed in later version.
         dialog.setOnShown(e ->
                           {
                              runLater(JavaFXMissingTools.class, () ->
                              {
                                 dialog.setX(owner.getX() + 0.5 * (owner.getWidth() - dialog.getWidth()));
                                 dialog.setY(owner.getY() + 0.5 * (owner.getHeight() - dialog.getHeight()));
                              });
                           });
      }
   }

   public static void centerWindowInOwner(Window window, Window owner)
   {
      window.setX(owner.getX() + 0.5 * (owner.getWidth() - window.getWidth()));
      window.setY(owner.getY() + 0.5 * (owner.getHeight() - window.getHeight()));

      if (!window.isShowing())
      {
         // TODO Seems that on Ubuntu the changes done to the window position/size are not processed properly until the window is showing.
         // This may be related to the bug reported when using GTK3: https://github.com/javafxports/openjdk-jfx/pull/446, might be fixed in later version.
         window.addEventHandler(WindowEvent.WINDOW_SHOWN, e ->
         {
            runLater(JavaFXMissingTools.class, () ->
            {
               window.setX(owner.getX() + 0.5 * (owner.getWidth() - window.getWidth()));
               window.setY(owner.getY() + 0.5 * (owner.getHeight() - window.getHeight()));
            });
         });
      }
   }

   public static void toEuclid(javafx.scene.transform.Transform jfxTransform, AffineTransform euclidTransform)
   {
      euclidTransform.set(jfxTransform.getMxx(),
                          jfxTransform.getMxy(),
                          jfxTransform.getMxz(),
                          jfxTransform.getTx(),
                          jfxTransform.getMyx(),
                          jfxTransform.getMyy(),
                          jfxTransform.getMyz(),
                          jfxTransform.getTy(),
                          jfxTransform.getMzx(),
                          jfxTransform.getMzy(),
                          jfxTransform.getMzz(),
                          jfxTransform.getTz());
   }

   public static void toJavaFX(RigidBodyTransformReadOnly euclidTransform, javafx.scene.transform.Affine jfxTransform)
   {
      if (euclidTransform instanceof RigidBodyTransform matrixTransform)
      {
         jfxTransform.setToTransform(matrixTransform.getM00(),
                                     matrixTransform.getM01(),
                                     matrixTransform.getM02(),
                                     matrixTransform.getM03(),
                                     matrixTransform.getM10(),
                                     matrixTransform.getM11(),
                                     matrixTransform.getM12(),
                                     matrixTransform.getM13(),
                                     matrixTransform.getM20(),
                                     matrixTransform.getM21(),
                                     matrixTransform.getM22(),
                                     matrixTransform.getM23());
      }
      else
      {
         toJavaFX(new RigidBodyTransform(euclidTransform), jfxTransform);
      }
   }

   public static javafx.geometry.Point3D toJavaFX(Tuple3DReadOnly euclidInput)
   {
      if (euclidInput == null)
         return null;
      else
         return new javafx.geometry.Point3D(euclidInput.getX(), euclidInput.getY(), euclidInput.getZ());
   }

   public static void transform(Transform transform, Point3DBasics pointToTransform)
   {
      transform(transform, pointToTransform, pointToTransform);
   }

   public static void transform(Transform transform, Point3DReadOnly pointOriginal, Tuple3DBasics pointTransformed)
   {
      transform(transform, pointOriginal, pointTransformed, true);
   }

   public static void transform(Transform transform, Vector3DBasics vectorToTransform)
   {
      transform(transform, vectorToTransform, vectorToTransform, false);
   }

   public static void transform(Transform transform, Vector3DReadOnly vectorOriginal, Tuple3DBasics vectorTransformed)
   {
      transform(transform, vectorOriginal, vectorTransformed, false);
   }

   public static void transform(Transform transform, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed, boolean applyTranslation)
   {
      double x_in = tupleOriginal.getX();
      double y_in = tupleOriginal.getY();
      double z_in = tupleOriginal.getZ();
      double x_out = transform.getMxx() * x_in + transform.getMxy() * y_in + transform.getMxz() * z_in;
      double y_out = transform.getMyx() * x_in + transform.getMyy() * y_in + transform.getMyz() * z_in;
      double z_out = transform.getMzx() * x_in + transform.getMzy() * y_in + transform.getMzz() * z_in;

      if (applyTranslation)
      {
         x_out += transform.getTx();
         y_out += transform.getTy();
         z_out += transform.getTz();
      }

      tupleTransformed.set(x_out, y_out, z_out);
   }

   public static void inverseTransform(Transform transform, Point3DBasics pointToTransform)
   {
      inverseTransform(transform, pointToTransform, pointToTransform);
   }

   public static void inverseTransform(Transform transform, Point3DReadOnly pointOriginal, Tuple3DBasics pointTransformed)
   {
      inverseTransform(transform, pointOriginal, pointTransformed, true);
   }

   public static void inverseTransform(Transform transform, Vector3DBasics vectorToTransform)
   {
      inverseTransform(transform, vectorToTransform, vectorToTransform, false);
   }

   public static void inverseTransform(Transform transform, Vector3DReadOnly vectorOriginal, Tuple3DBasics vectorTransformed)
   {
      inverseTransform(transform, vectorOriginal, vectorTransformed, false);
   }

   public static void inverseTransform(Transform transform, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed, boolean applyTranslation)
   {
      double x_in = tupleOriginal.getX();
      double y_in = tupleOriginal.getY();
      double z_in = tupleOriginal.getZ();
      double x_out, y_out, z_out;

      if (transform instanceof Rotate)
      {
         double m00 = transform.getMxx();
         double m11 = transform.getMyy();
         double m21 = transform.getMzy();
         double m20 = transform.getMzx();
         double m10 = transform.getMyx();
         double m01 = transform.getMxy();
         double m12 = transform.getMyz();
         double m22 = transform.getMzz();
         double m02 = transform.getMxz();

         x_out = m00 * x_in + m10 * y_in + m20 * z_in;
         y_out = m01 * x_in + m11 * y_in + m21 * z_in;
         z_out = m02 * x_in + m12 * y_in + m22 * z_in;
      }
      else if (transform instanceof Translate)
      {
         if (applyTranslation)
         {
            x_out = x_in - transform.getTx();
            y_out = y_in - transform.getTy();
            z_out = z_in - transform.getTz();
         }
         else
         {
            x_out = x_in;
            y_out = y_in;
            z_out = z_in;
         }
      }
      else if (transform instanceof Scale)
      {
         Scale scale = ((Scale) transform);
         x_out = x_in / scale.getX();
         y_out = y_in / scale.getY();
         z_out = z_in / scale.getZ();
      }
      else
      {
         double det = transform.determinant();
         if (det == 0.0) // To be consistent with JavaFX.
            throw new SingularMatrixException("Determinant is 0");

         double m00 = transform.getMxx();
         double m11 = transform.getMyy();
         double m21 = transform.getMzy();
         double m20 = transform.getMzx();
         double m10 = transform.getMyx();
         double m01 = transform.getMxy();
         double m12 = transform.getMyz();
         double m22 = transform.getMzz();
         double m02 = transform.getMxz();

         det = 1.0 / det;
         double invM00 = (m11 * m22 - m21 * m12) * det;
         double invM01 = -(m01 * m22 - m21 * m02) * det;
         double invM02 = (m01 * m12 - m11 * m02) * det;
         double invM10 = -(m10 * m22 - m20 * m12) * det;
         double invM11 = (m00 * m22 - m20 * m02) * det;
         double invM12 = -(m00 * m12 - m10 * m02) * det;
         double invM20 = (m10 * m21 - m20 * m11) * det;
         double invM21 = -(m00 * m21 - m20 * m01) * det;
         double invM22 = (m00 * m11 - m10 * m01) * det;

         double x_tem = x_in;
         double y_tem = y_in;
         double z_tem = z_in;

         if (applyTranslation)
         {
            x_tem = x_in - transform.getTx();
            y_tem = y_in - transform.getTy();
            z_tem = z_in - transform.getTz();
         }
         else
         {
            x_tem = x_in;
            y_tem = y_in;
            z_tem = z_in;
         }
         x_out = invM00 * x_tem + invM01 * y_tem + invM02 * z_tem;
         y_out = invM10 * x_tem + invM11 * y_tem + invM12 * z_tem;
         z_out = invM20 * x_tem + invM21 * y_tem + invM22 * z_tem;
      }

      tupleTransformed.set(x_out, y_out, z_out);
   }

   public static void setDrawModeRecursive(Node start, DrawMode drawMode)
   {
      if (start instanceof Shape3D)
         ((Shape3D) start).setDrawMode(drawMode);
      if (start instanceof Group)
         ((Group) start).getChildren().forEach(c -> setDrawModeRecursive(c, drawMode));
   }
}
