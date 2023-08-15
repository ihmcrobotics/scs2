package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import java.util.function.Predicate;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableDoubleValue;
import javafx.event.EventHandler;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

/**
 * This handles the camera position and orientation such that:
 * <ul>
 * <li>the camera is always fixing at a focal point located at (0, 0, 0).
 * <li>the camera is a desired offset from the focal point.
 * <li>the roll of the camera is directly controlled.
 * </ul>
 *
 * @author Sylvain Bertrand
 */
public class CameraOrbitHandler
{

   /**
    * The current pose of the camera defining its orientation and distance from the focal point located
    * at (0, 0, 0).
    */
   private final Affine cameraPose = new Affine();
   /**
    * Rotation offset computed such that when the {@link #latitude}, {@link #longitude}, and
    * {@link #roll} are all zero, the camera is orientated as follows:
    * <li>the viewing direction (e.g. z-axis) of the camera is aligned with the given forward axis.
    * <li>the vertical direction of the screen (e.g. y-axis) is collinear with the given up axis.
    */
   private final Affine offset = new Affine();

   private final Property<CameraControlMode> controlMode = new SimpleObjectProperty<>(this, "controlMode", CameraControlMode.Orbital);

   /**
    * Latitude of the camera on the virtual sphere:
    * <li>the latitude is in the range [-90 degrees; +90 degrees],
    * <li>a latitude of +90 degrees corresponds to the camera located at the north pole looking
    * straight down,
    * <li>a latitude of -90 degrees corresponds to the camera located at the south pole looking
    * straight up.
    */
   private final DoubleProperty latitude = new SimpleDoubleProperty(this, "latitude", 0.0);
   /**
    * Longitude of the camera on the virtual sphere:
    * <li>the longitude is in the range [-180 degrees; +180 degrees],
    * <li>at a longitude of zero degree, the camera is directed torward the forward axis.
    */
   private final DoubleProperty longitude = new SimpleDoubleProperty(this, "longitude", 0.0);
   /**
    * Roll of the camera. When it is zero, the camera is leveled.
    */
   private final DoubleProperty roll = new SimpleDoubleProperty(this, "roll", 0.0);
   /**
    * When set to true, the camera roll is zeroed to keep the camera level.
    */
   private final BooleanProperty keepRotationLeveled = new SimpleBooleanProperty(this, "keepRotationLeveled", true);
   /**
    * Condition to trigger the use of the fast modifier to make the camera rotate faster when using the
    * mouse.
    */
   private final ObjectProperty<Predicate<MouseEvent>> fastModifierPredicate = new SimpleObjectProperty<>(this, "fastModifierPredicate", null);
   /** Slow camera rotation modifier when using the mouse. */
   private final DoubleProperty slowModifier = new SimpleDoubleProperty(this, "slowModifier", 0.005);
   /**
    * Fast camera rotation modifier when using the mouse. It is triggered when the condition held in
    * {@link #fastModifierPredicate} is fulfilled.
    */
   private final DoubleProperty fastModifier = new SimpleDoubleProperty(this, "fastModifier", 0.010);
   /** Camera roll modifier when using the mouse. */
   private final DoubleProperty rollModifier = new SimpleDoubleProperty(this, "rollModifier", 0.005);
   /**
    * Indicates which mouse button triggers the camera rotation when using the {@link EventHandler} via
    * {@link #createMouseEventHandler(ReadOnlyDoubleProperty, ReadOnlyDoubleProperty)}.
    */
   private final ObjectProperty<MouseButton> rotationMouseButton = new SimpleObjectProperty<>(this, "rotationMouseButton", MouseButton.PRIMARY);

   /**
    * When set to true, the latitude of the camera is restricted to the range [{@link #minLatitude},
    * {@link #maxLatitude}].
    */
   private final BooleanProperty restrictLatitude = new SimpleBooleanProperty(this, "restrictLatitude", true);
   /** Minimum latitude of the camera. Only used when {@link #restrictLatitude} is set to true. */
   private final DoubleProperty minLatitude = new SimpleDoubleProperty(this, "minLatitude", -Math.PI / 2.0 + 0.01);
   /** Maximum latitude of the camera. Only used when {@link #restrictLatitude} is set to true. */
   private final DoubleProperty maxLatitude = new SimpleDoubleProperty(this, "maxLatitude", Math.PI / 2.0 - 0.01);

   /**
    * Distance between camera and focal point, it serves for zooming in and out.
    */
   private final DoubleProperty distance = new SimpleDoubleProperty(this, "distance", 10.0);
   /** Minimum value the zoom can be. */
   private final DoubleProperty minDistance = new SimpleDoubleProperty(this, "minDistance", 0.1);
   /** Maximum value the zoom can be. */
   private final DoubleProperty maxDistance = new SimpleDoubleProperty(this, "maxDistance", 100.0);

   private final DoubleProperty x = new SimpleDoubleProperty(this, "xLocal", 0);
   private final DoubleProperty y = new SimpleDoubleProperty(this, "yLocal", 0);
   private final DoubleProperty z = new SimpleDoubleProperty(this, "zLocal", 0);
   private DoubleProperty xWorld;
   private DoubleProperty yWorld;
   private DoubleProperty zWorld;

   /**
    * Zoom speed factor with respect to its current value. The larger is the zoom, the faster it
    * "goes".
    */
   private final DoubleProperty distanceModifier = new SimpleDoubleProperty(this, "distanceModifier", -0.1);

   private final Vector3DReadOnly down;
   private final Vector3DReadOnly forward;

   private boolean disableCameraPoseAutoUpdate = false;

   /**
    * Creates a calculator for the camera rotation.
    *
    * @param up      indicates which way is up.
    * @param forward indicates which is forward.
    * @throws RuntimeException if {@code up} and {@code forward} are not orthogonal.
    */
   public CameraOrbitHandler(Vector3DReadOnly up, Vector3DReadOnly forward)
   {
      Vector3D left = new Vector3D();
      left.cross(up, forward);
      if (!MathTools.epsilonEquals(left.norm(), 1.0, Epsilons.ONE_HUNDRED_THOUSANDTH))
         throw new RuntimeException("The vectors up and forward must be orthogonal. Received: up = " + up + ", forward = " + forward);

      this.forward = forward;
      down = EuclidCoreFactories.newNegativeLinkedVector3D(up);

      computeOffset();

      longitude.addListener((o, oldValue, newValue) ->
      {
         if (disableCameraPoseAutoUpdate)
            return;
         if (controlMode.getValue() == CameraControlMode.Orbital)
         {
            setOrbit(Double.NaN, newValue.doubleValue(), Double.NaN, Double.NaN);
         }
         else if (controlMode.getValue() == CameraControlMode.LevelOrbital)
         {
            setLevelOrbit(Double.NaN, newValue.doubleValue(), Double.NaN, Double.NaN);
         }
         else if (!longitude.isBound())
         {
            boolean disablePrevious = disableCameraPoseAutoUpdate;
            disableCameraPoseAutoUpdate = true;
            longitude.setValue(oldValue);
            disableCameraPoseAutoUpdate = disablePrevious;
         }
      });
      latitude.addListener((o, oldValue, newValue) ->
      {
         if (disableCameraPoseAutoUpdate)
            return;
         if (controlMode.getValue() == CameraControlMode.Orbital)
         {
            setOrbit(Double.NaN, Double.NaN, newValue.doubleValue(), Double.NaN);
         }
         else if (!latitude.isBound())
         {
            boolean disablePrevious = disableCameraPoseAutoUpdate;
            disableCameraPoseAutoUpdate = true;
            latitude.setValue(oldValue);
            disableCameraPoseAutoUpdate = disablePrevious;
         }
      });
      roll.addListener((o, oldValue, newValue) ->
      {
         if (!disableCameraPoseAutoUpdate)
            setOrbit(Double.NaN, Double.NaN, Double.NaN, newValue.doubleValue());
      });

      x.addListener((o, oldValue, newValue) ->
      {
         if (disableCameraPoseAutoUpdate)
            return;
         if (controlMode.getValue() == CameraControlMode.Position)
         {
            setPosition(newValue.doubleValue(), Double.NaN, Double.NaN, Double.NaN);
         }
         else if (!isXBound())
         {
            boolean disablePrevious = disableCameraPoseAutoUpdate;
            disableCameraPoseAutoUpdate = true;
            x.set(oldValue.doubleValue());
            disableCameraPoseAutoUpdate = disablePrevious;
         }
      });
      y.addListener((o, oldValue, newValue) ->
      {
         if (disableCameraPoseAutoUpdate)
            return;
         if (controlMode.getValue() == CameraControlMode.Position)
         {
            setPosition(Double.NaN, newValue.doubleValue(), Double.NaN, Double.NaN);
         }
         else if (!isYBound())
         {
            boolean disablePrevious = disableCameraPoseAutoUpdate;
            disableCameraPoseAutoUpdate = true;
            y.setValue(oldValue);
            disableCameraPoseAutoUpdate = disablePrevious;
         }
      });
      z.addListener((o, oldValue, newValue) ->
      {
         if (disableCameraPoseAutoUpdate)
            return;
         if (controlMode.getValue() == CameraControlMode.Position)
         {
            setPosition(Double.NaN, Double.NaN, newValue.doubleValue(), Double.NaN);
         }
         else if (controlMode.getValue() == CameraControlMode.LevelOrbital)
         {
            setLevelOrbit(Double.NaN, Double.NaN, newValue.doubleValue(), Double.NaN);
         }
         else if (!isZBound())
         {
            boolean disablePrevious = disableCameraPoseAutoUpdate;
            disableCameraPoseAutoUpdate = true;
            z.setValue(oldValue);
            disableCameraPoseAutoUpdate = disablePrevious;
         }
      });

      distance.addListener((o, oldValue, newValue) ->
      {
         if (disableCameraPoseAutoUpdate)
            return;
         if (controlMode.getValue() == CameraControlMode.Orbital)
         {
            setOrbit(newValue.doubleValue(), Double.NaN, Double.NaN, Double.NaN);
         }
         else if (controlMode.getValue() == CameraControlMode.LevelOrbital)
         {
            setLevelOrbit(newValue.doubleValue(), Double.NaN, Double.NaN, Double.NaN);
         }
         else if (!distance.isBound())
         {
            boolean disablePrevious = disableCameraPoseAutoUpdate;
            disableCameraPoseAutoUpdate = true;
            distance.setValue(oldValue);
            disableCameraPoseAutoUpdate = disablePrevious;
         }
      });
   }

   private boolean isXBound()
   {
      return x.isBound() || (xWorld != null && xWorld.isBound());
   }

   private boolean isYBound()
   {
      return y.isBound() || (yWorld != null && yWorld.isBound());
   }

   private boolean isZBound()
   {
      return z.isBound() || (zWorld != null && zWorld.isBound());
   }

   private void computeOffset()
   {
      Vector3D cameraXAxis = new Vector3D();
      cameraXAxis.cross(down, forward);
      RotationMatrix rotationOffset = new RotationMatrix();
      rotationOffset.setColumns(cameraXAxis, down, forward);
      JavaFXMissingTools.convertRotationMatrixToAffine(rotationOffset, offset);
   }

   /**
    * Creates an {@link EventHandler} to rotate the camera when a mouse click and drag is performed.
    *
    * @param sceneWidthProperty  width of the scene the camera belongs to.
    * @param sceneHeightProperty height of the scene the camera belongs to.
    * @return an {@link EventHandler} to rotate the camera with the mouse.
    */
   public EventHandler<MouseEvent> createMouseEventHandler(ReadOnlyDoubleProperty sceneWidthProperty, ReadOnlyDoubleProperty sceneHeightProperty)
   {
      return new EventHandler<>()
      {
         private final Point2D oldMouseLocation = new Point2D();

         @Override
         public void handle(MouseEvent event)
         {
            if (event.getButton() != rotationMouseButton.get())
               return;

            // Filters single clicks
            if (event.isStillSincePress())
               return;

            if (event.getEventType() == MouseEvent.MOUSE_PRESSED)
            {
               oldMouseLocation.set(event.getSceneX(), event.getSceneY());
               return;
            }

            if (event.getEventType() != MouseEvent.MOUSE_DRAGGED)
               return;

            Point2D centerLocation = new Point2D();
            Point2D newMouseLocation = new Point2D();
            // Acquire the new mouse coordinates from the recent event
            centerLocation.set(sceneWidthProperty.get() / 2.0, sceneHeightProperty.get() / 2.0);
            newMouseLocation.set(event.getSceneX(), event.getSceneY());

            double modifier;
            if (fastModifierPredicate.get() == null || !fastModifierPredicate.get().test(event))
               modifier = slowModifier.get();
            else
               modifier = fastModifier.get();

            Vector2D drag = new Vector2D();
            drag.sub(newMouseLocation, oldMouseLocation);

            double rollShift;
            if (keepRotationLeveled.get())
            {
               rollShift = Double.NaN;
            }
            else
            {
               Vector2D centerToMouseLocation = new Vector2D();
               centerToMouseLocation.sub(newMouseLocation, centerLocation);
               rollShift = modifier * rollModifier.get() * drag.cross(centerToMouseLocation);
            }

            drag.scale(modifier);
            switch (controlMode.getValue())
            {
               case Position:
               {
                  double dxy = Math.cos(latitude.get() + drag.getY()) * distance.get();
                  setPosition(-Math.cos(longitude.get() - drag.getX()) * dxy,
                              -Math.sin(longitude.get() - drag.getX()) * dxy,
                              Math.sin(latitude.get() + drag.getY()) * distance.get(),
                              roll.get() + rollShift);
                  break;
               }
               case Orbital:
               {
                  setOrbit(Double.NaN, longitude.get() - drag.getX(), latitude.get() + drag.getY(), roll.get() + rollShift);
                  break;
               }
               case LevelOrbital:
               {
                  setLevelOrbit(Double.NaN, longitude.get() - drag.getX(), z.get() + Math.sin(drag.getY()) * distance.get(), roll.get() + rollShift);
                  break;
               }
               default:
               {
                  throw new IllegalArgumentException("Unexpected value: " + controlMode.getValue());
               }
            }

            oldMouseLocation.set(newMouseLocation);
         }
      };
   }

   /**
    * @return an {@link EventHandler} for {@link ScrollEvent} that uses the mouse wheel to update the
    *         zoom value.
    */
   public EventHandler<ScrollEvent> createScrollEventHandler()
   {
      return new EventHandler<>()
      {
         @Override
         public void handle(ScrollEvent event)
         {
            if (distance.isBound())
               return;

            double direction = Math.signum(event.getDeltaY());
            double newDistance = distance.get() + direction * distance.get() * distanceModifier.get();

            switch (controlMode.getValue())
            {
               case Position:
               {
                  newDistance = MathTools.clamp(newDistance, minDistance.get(), maxDistance.get());
                  double scale = newDistance / distance.get();
                  setPosition(x.get() * scale, y.get() * scale, z.get() * scale, Double.NaN);
                  break;
               }
               case Orbital:
               {
                  setOrbit(newDistance, Double.NaN, Double.NaN, Double.NaN);
                  break;
               }
               case LevelOrbital:
               {
                  // Changing the distance to zoom in/out is expected to preserve the latitude not the height.
                  newDistance = MathTools.clamp(newDistance, minDistance.get(), maxDistance.get());
                  double newHeight = z.get() + (newDistance - distance.get()) * Math.sin(latitude.get());
                  setLevelOrbit(newDistance, Double.NaN, newHeight, Double.NaN);
                  break;
               }
               default:
               {
                  throw new IllegalArgumentException("Unexpected value: " + controlMode.getValue());
               }
            }
         }
      };
   }

   /**
    * Update the camera rotation after applying rotation offsets.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param deltaLongitude the shift in longitude to apply to the camera rotation.
    * @param deltaLatitude  the shift in latitude to apply to the camera rotation.
    * @param deltaRoll      the shift in roll to apply to the camera rotation.
    */
   public void rotate(double deltaLongitude, double deltaLatitude, double deltaRoll)
   {
      rotate(deltaLongitude, deltaLatitude, deltaRoll, false);
   }

   /**
    * Update the camera rotation after applying rotation offsets.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param deltaLongitude the shift in longitude to apply to the camera rotation.
    * @param deltaLatitude  the shift in latitude to apply to the camera rotation.
    * @param deltaRoll      the shift in roll to apply to the camera rotation.
    * @return the amount to translate the focal point to use for making the camera rotate on itself
    *         instead of orbiting around a fixed focal point.
    */
   public Vector3DReadOnly rotate(double deltaLongitude, double deltaLatitude, double deltaRoll, boolean computeFocalPointShift)
   {
      return setRotation(longitude.get() + deltaLongitude, latitude.get() + deltaLatitude, roll.get() + deltaRoll, computeFocalPointShift);
   }

   /**
    * Computes the camera orientation such that the camera position lands at the desired position
    * without changing the focal point.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param position desired camera position. Not modified.
    * @param roll     desired camera roll.
    */
   public void setPosition(Point3DReadOnly position, double roll)
   {
      setPosition(position.getX(), position.getY(), position.getZ(), roll);
   }

   /**
    * Computes the camera orientation such that the camera position lands at the desired position
    * without changing the focal point.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param x    the desired camera x-coordinate.
    * @param y    the desired camera y-coordinate.
    * @param z    the desired camera z-coordinate.
    * @param roll desired camera roll.
    */
   public void setPosition(double x, double y, double z, double roll)
   {
      if (this.distance.isBound())
      {
         LogTools.error("Cannot set position, distance coordinate is bound.");
         return;
      }
      if (this.longitude.isBound())
      {
         LogTools.error("Cannot set position, longitude coordinate is bound.");
         return;
      }
      if (this.latitude.isBound())
      {
         LogTools.error("Cannot set position, latitude coordinate is bound.");
         return;
      }

      disableCameraPoseAutoUpdate = true;

      if (!Double.isFinite(x) || isXBound())
         x = this.x.get();
      else
         this.x.set(x);
      if (!Double.isFinite(y) || isYBound())
         y = this.y.get();
      else
         this.y.set(y);
      if (!Double.isFinite(z) || isZBound())
         z = this.z.get();
      else
         this.z.set(z);

      double distance = EuclidCoreTools.norm(x, y, z);
      double longitude = Math.atan2(-y, -x);
      double latitude = Math.atan(z / EuclidCoreTools.norm(x, y));

      this.distance.set(distance);
      this.longitude.set(longitude);
      this.latitude.set(latitude);

      if (keepRotationLeveled.get() && !this.roll.isBound())
      {
         roll = 0;
         this.roll.set(0);
      }
      else if (Double.isFinite(roll) && !this.roll.isBound())
      {
         roll = EuclidCoreTools.trimAngleMinusPiToPi(roll);
         this.roll.set(roll);
      }
      else
      {
         roll = this.roll.get();
      }

      Affine newPose = new Affine();
      newPose.append(offset);
      newPose.append(new Rotate(Math.toDegrees(-longitude), Rotate.Y_AXIS));
      newPose.append(new Rotate(Math.toDegrees(-latitude), Rotate.X_AXIS));
      if (roll != 0.0)
         newPose.append(new Rotate(Math.toDegrees(roll), Rotate.Z_AXIS));
      newPose.setTx(x);
      newPose.setTy(y);
      newPose.setTz(z);
      cameraPose.setToTransform(newPose);

      disableCameraPoseAutoUpdate = false;
   }

   /**
    * Sets the camera's latitude, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param longitude the new camera longitude.
    * @param latitude  the new camera latitude.
    * @param roll      the new camera roll.
    */
   public void setRotation(double longitude, double latitude, double roll)
   {
      setRotation(longitude, latitude, roll, false);
   }

   /**
    * Sets the camera's latitude, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param longitude the new camera longitude.
    * @param latitude  the new camera latitude.
    * @param roll      the new camera roll.
    * @return the amount to translate the focal point to use for making the camera rotate on itself
    *         instead of orbiting around a fixed focal point.
    */
   public Vector3DReadOnly setRotation(double longitude, double latitude, double roll, boolean computeFocalPointShift)
   {
      return setOrbit(Double.NaN, longitude, latitude, roll, computeFocalPointShift);
   }

   /**
    * Sets the camera's distance from focal point, latitude, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param distance  the new distance between camera and focal point.
    * @param longitude the new camera longitude.
    * @param latitude  the new camera latitude.
    * @param roll      the new camera roll.
    */
   public void setOrbit(double distance, double longitude, double latitude, double roll)
   {
      setOrbit(distance, longitude, latitude, roll, false);
   }

   /**
    * Sets the camera's distance from focal point, latitude, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param distance  the new distance between camera and focal point.
    * @param longitude the new camera longitude.
    * @param latitude  the new camera latitude.
    * @param roll      the new camera roll.
    * @return the amount to translate the focal point to use for making the camera rotate on itself
    *         instead of orbiting around a fixed focal point.
    */
   public Vector3DReadOnly setOrbit(double distance, double longitude, double latitude, double roll, boolean computeFocalPointShift)
   {
      if (isXBound())
      {
         LogTools.error("Cannot set orbit, x coordinate is bound.");
         return EuclidCoreTools.zeroVector3D;
      }
      if (isYBound())
      {
         LogTools.error("Cannot set orbit, y coordinate is bound.");
         return EuclidCoreTools.zeroVector3D;
      }
      if (isZBound())
      {
         LogTools.error("Cannot set orbit, z coordinate is bound.");
         return EuclidCoreTools.zeroVector3D;
      }

      disableCameraPoseAutoUpdate = true;

      if (Double.isFinite(distance) && !this.distance.isBound())
      {
         distance = MathTools.clamp(distance, minDistance.get(), maxDistance.get());
         this.distance.set(distance);
      }
      else
      {
         distance = this.distance.get();
      }

      if (Double.isFinite(latitude) && !this.latitude.isBound())
      {
         if (restrictLatitude.get())
            latitude = MathTools.clamp(latitude, minLatitude.get(), maxLatitude.get());
         else
            latitude = MathTools.clamp(latitude, Math.PI / 2.0);
         this.latitude.set(latitude);
      }
      else
      {
         latitude = this.latitude.get();
      }

      if (Double.isFinite(longitude) && !this.longitude.isBound())
      {
         longitude = EuclidCoreTools.trimAngleMinusPiToPi(longitude);
         this.longitude.set(longitude);
      }
      else
      {
         longitude = this.longitude.get();
      }

      if (keepRotationLeveled.get() && !this.roll.isBound())
      {
         roll = 0;
         this.roll.set(0);
      }
      else if (Double.isFinite(roll) && !this.roll.isBound())
      {
         roll = EuclidCoreTools.trimAngleMinusPiToPi(roll);
         this.roll.set(roll);
      }
      else
      {
         roll = this.roll.get();
      }

      Vector3D focalPointTranslation = null;

      if (computeFocalPointShift)
         focalPointTranslation = new Vector3D(cameraPose.getTx(), cameraPose.getTy(), cameraPose.getTz());

      Affine newPose = new Affine();
      newPose.append(offset);
      newPose.append(new Rotate(Math.toDegrees(-longitude), Rotate.Y_AXIS));
      newPose.append(new Rotate(Math.toDegrees(-latitude), Rotate.X_AXIS));
      if (roll != 0.0)
         newPose.append(new Rotate(Math.toDegrees(roll), Rotate.Z_AXIS));
      // we need to shift the camera backward
      newPose.setTx(-distance * newPose.getMxz());
      newPose.setTy(-distance * newPose.getMyz());
      newPose.setTz(-distance * newPose.getMzz());
      cameraPose.setToTransform(newPose);

      if (computeFocalPointShift)
         focalPointTranslation.sub(cameraPose.getTx(), cameraPose.getTy(), cameraPose.getTz());

      x.set(cameraPose.getTx());
      y.set(cameraPose.getTy());
      z.set(cameraPose.getTz());

      disableCameraPoseAutoUpdate = false;
      return focalPointTranslation;
   }

   /**
    * Sets the camera's distance from focal point, height, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param distance  the new distance between camera and focal point.
    * @param longitude the new camera longitude.
    * @param height    the new camera height.
    * @param roll      the new camera roll.
    */
   public void setLevelOrbit(double distance, double longitude, double height, double roll)
   {
      setLevelOrbit(distance, longitude, height, roll, false);
   }

   /**
    * Sets the camera's distance from focal point, height, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param distance  the new distance between camera and focal point.
    * @param longitude the new camera longitude.
    * @param height    the new camera height.
    * @param roll      the new camera roll.
    * @return the amount to translate the focal point to use for making the camera rotate on itself
    *         instead of orbiting around a fixed focal point.
    */
   public Vector3DReadOnly setLevelOrbit(double distance, double longitude, double height, double roll, boolean computeFocalPointShift)
   {
      if (isXBound())
      {
         LogTools.error("Cannot set level-orbit, x coordinate is bound.");
         return EuclidCoreTools.zeroVector3D;
      }
      if (isYBound())
      {
         LogTools.error("Cannot set level-orbit, y coordinate is bound.");
         return EuclidCoreTools.zeroVector3D;
      }
      if (latitude.isBound())
      {
         LogTools.error("Cannot set level-orbit, latitude coordinate is bound.");
         return EuclidCoreTools.zeroVector3D;
      }

      disableCameraPoseAutoUpdate = true;

      if (Double.isFinite(distance) && !this.distance.isBound())
      {
         distance = MathTools.clamp(distance, minDistance.get(), maxDistance.get());
         this.distance.set(distance);
      }
      else
      {
         distance = this.distance.get();
      }

      if (!Double.isFinite(height) || isZBound())
      {
         height = z.get();

         // Cannot use the height to control the min/max latitude, so using the distance instead.
         if (!this.distance.isBound())
         {
            if (restrictLatitude.get())
            {
               if (height > 0.0)
                  distance = Math.max(distance, height / Math.sin(maxLatitude.get()));
               else
                  distance = Math.max(distance, height / Math.sin(minLatitude.get()));
            }
            else
            {
               distance = Math.max(distance, height);
            }
            this.distance.set(distance);
         }
      }
      else
      {
         if (restrictLatitude.get())
            height = MathTools.clamp(height, Math.sin(minLatitude.get()) * distance, Math.sin(maxLatitude.get()) * distance);
         z.set(height);
      }

      double latitude = Math.asin(MathTools.clamp(height / distance, 1.0));
      this.latitude.set(latitude);

      if (Double.isFinite(longitude) && !this.longitude.isBound())
      {
         longitude = EuclidCoreTools.trimAngleMinusPiToPi(longitude);
         this.longitude.set(longitude);
      }
      else
      {
         longitude = this.longitude.get();
      }

      if (keepRotationLeveled.get() && !this.roll.isBound())
      {
         roll = 0;
         this.roll.set(0);
      }
      else if (Double.isFinite(roll) && !this.roll.isBound())
      {
         roll = EuclidCoreTools.trimAngleMinusPiToPi(roll);
         this.roll.set(roll);
      }
      else
      {
         roll = this.roll.get();
      }

      Vector3D focalPointTranslation = null;

      if (computeFocalPointShift)
         focalPointTranslation = new Vector3D(cameraPose.getTx(), cameraPose.getTy(), cameraPose.getTz());

      Affine newPose = new Affine();
      newPose.append(offset);
      newPose.append(new Rotate(Math.toDegrees(-longitude), Rotate.Y_AXIS));
      newPose.append(new Rotate(Math.toDegrees(-latitude), Rotate.X_AXIS));
      if (roll != 0.0)
         newPose.append(new Rotate(Math.toDegrees(roll), Rotate.Z_AXIS));
      // we need to shift the camera backward
      newPose.setTx(-distance * newPose.getMxz());
      newPose.setTy(-distance * newPose.getMyz());
      newPose.setTz(-distance * newPose.getMzz());
      cameraPose.setToTransform(newPose);

      if (computeFocalPointShift)
         focalPointTranslation.sub(cameraPose.getTx(), cameraPose.getTy(), cameraPose.getTz());

      x.set(cameraPose.getTx());
      y.set(cameraPose.getTy());

      disableCameraPoseAutoUpdate = false;
      return focalPointTranslation;
   }

   public Vector3DReadOnly computeFocalPointShift(double deltaDistance, double deltaLongitude, double deltaLatitude, double deltaRoll)
   {
      if (!Double.isFinite(deltaDistance))
         deltaDistance = 0.0;
      if (!Double.isFinite(deltaLongitude))
         deltaLongitude = 0.0;
      if (!Double.isFinite(deltaLatitude))
         deltaLatitude = 0.0;
      if (!Double.isFinite(deltaRoll))
         deltaRoll = 0.0;

      if (deltaDistance == 0.0 && deltaLongitude == 0.0 && deltaLatitude == 0.0)
         return EuclidCoreTools.zeroVector3D;

      double distance = deltaDistance + this.distance.get();
      double longitude = deltaLongitude + this.longitude.get();
      double latitude = deltaLatitude + this.latitude.get();
      double roll = deltaRoll + this.roll.get();

      Affine newPose = new Affine();
      newPose.append(offset);
      newPose.append(new Rotate(Math.toDegrees(-longitude), Rotate.Y_AXIS));
      newPose.append(new Rotate(Math.toDegrees(-latitude), Rotate.X_AXIS));
      if (roll != 0.0)
         newPose.append(new Rotate(Math.toDegrees(roll), Rotate.Z_AXIS));
      // we need to shift the camera backward
      newPose.setTx(-distance * newPose.getMxz());
      newPose.setTy(-distance * newPose.getMyz());
      newPose.setTz(-distance * newPose.getMzz());

      Vector3D focalPointTranslation = new Vector3D(cameraPose.getTx(), cameraPose.getTy(), cameraPose.getTz());
      focalPointTranslation.sub(newPose.getTx(), newPose.getTy(), newPose.getTz());
      return focalPointTranslation;
   }

   public Tuple3DProperty createCameraWorldCoordinates(ObservableDoubleValue xOffset, ObservableDoubleValue yOffset, ObservableDoubleValue zOffset)
   {
      if (xWorld != null)
         return new Tuple3DProperty(xWorld, yWorld, zWorld);

      xWorld = new SimpleDoubleProperty(this, "xWorld", 0);
      yWorld = new SimpleDoubleProperty(this, "yWorld", 0);
      zWorld = new SimpleDoubleProperty(this, "zWorld", 0);
      DoubleProperty[] cameraWorldCoordinates = {xWorld, yWorld, zWorld};
      DoubleProperty[] orbitHandlerCartesianCoordinates = {x, y, z};
      ObservableDoubleValue[] offsets = {xOffset, yOffset, zOffset};

      for (int i = 0; i < 2; i++)
      { // Doing x and y coordinates only, z is slightly different because of the level-orbit mode
         MutableBoolean updating = new MutableBoolean(false);

         DoubleProperty orbitHandlerCartesianCoordinate = orbitHandlerCartesianCoordinates[i];
         ObservableDoubleValue offset = offsets[i];
         DoubleProperty cameraWorldCoordinate = cameraWorldCoordinates[i];

         cameraWorldCoordinate.addListener((o, oldValue, newValue) ->
         {
            if (updating.isTrue())
               return;
            updating.setTrue();
            orbitHandlerCartesianCoordinate.set(cameraWorldCoordinate.get() - offset.get());
            updating.setFalse();
         });
         offset.addListener((o, oldValue, newValue) ->
         {
            updating.setTrue();
            if (controlMode.getValue() == CameraControlMode.Position)
               orbitHandlerCartesianCoordinate.set(cameraWorldCoordinate.get() - offset.get());
            else
               cameraWorldCoordinate.set(orbitHandlerCartesianCoordinate.get() + offset.get());
            updating.setFalse();
         });
         orbitHandlerCartesianCoordinate.addListener((o, oldValue, newValue) ->
         {
            if (updating.isTrue())
               return;
            updating.setTrue();
            cameraWorldCoordinate.set(orbitHandlerCartesianCoordinate.get() + offset.get());
            updating.setFalse();
         });
      }

      {
         MutableBoolean updating = new MutableBoolean(false);

         DoubleProperty orbitHandlerCartesianCoordinate = orbitHandlerCartesianCoordinates[2];
         ObservableDoubleValue offset = offsets[2];
         DoubleProperty cameraWorldCoordinate = cameraWorldCoordinates[2];

         cameraWorldCoordinate.addListener((o, oldValue, newValue) ->
         {
            if (updating.isTrue())
               return;
            updating.setTrue();
            orbitHandlerCartesianCoordinate.set(cameraWorldCoordinate.get() - offset.get());
            updating.setFalse();
         });
         offset.addListener((o, oldValue, newValue) ->
         {
            updating.setTrue();
            if (controlMode.getValue() == CameraControlMode.Orbital)
               cameraWorldCoordinate.set(orbitHandlerCartesianCoordinate.get() + offset.get());
            else
               orbitHandlerCartesianCoordinate.set(cameraWorldCoordinate.get() - offset.get());
            updating.setFalse();
         });
         orbitHandlerCartesianCoordinate.addListener((o, oldValue, newValue) ->
         {
            if (updating.isTrue())
               return;
            updating.setTrue();
            cameraWorldCoordinate.set(orbitHandlerCartesianCoordinate.get() + offset.get());
            updating.setFalse();
         });
      }

      return new Tuple3DProperty(cameraWorldCoordinates);
   }

   public Affine getCameraPose()
   {
      return cameraPose;
   }

   public Property<CameraControlMode> controlMode()
   {
      return controlMode;
   }

   public final DoubleProperty latitudeProperty()
   {
      return latitude;
   }

   public final DoubleProperty longitudeProperty()
   {
      return longitude;
   }

   public final DoubleProperty rollProperty()
   {
      return roll;
   }

   public DoubleProperty xProperty()
   {
      return x;
   }

   public DoubleProperty yProperty()
   {
      return y;
   }

   public DoubleProperty zProperty()
   {
      return z;
   }

   public DoubleProperty xWorldProperty()
   {
      return xWorld;
   }

   public DoubleProperty yWorldProperty()
   {
      return yWorld;
   }

   public DoubleProperty zWorldProperty()
   {
      return zWorld;
   }

   public final BooleanProperty keepRotationLeveledProperty()
   {
      return keepRotationLeveled;
   }

   public final ObjectProperty<Predicate<MouseEvent>> fastModifierPredicateProperty()
   {
      return fastModifierPredicate;
   }

   public final DoubleProperty slowModifierProperty()
   {
      return slowModifier;
   }

   public final DoubleProperty fastModifierProperty()
   {
      return fastModifier;
   }

   public final DoubleProperty rollModifierProperty()
   {
      return rollModifier;
   }

   public final ObjectProperty<MouseButton> rotationMouseButtonProperty()
   {
      return rotationMouseButton;
   }

   public final BooleanProperty restrictLatitudeProperty()
   {
      return restrictLatitude;
   }

   public final DoubleProperty minLatitudeProperty()
   {
      return minLatitude;
   }

   public final DoubleProperty maxLatitudeProperty()
   {
      return maxLatitude;
   }

   public final DoubleProperty distanceProperty()
   {
      return distance;
   }

   public final DoubleProperty minDistanceProperty()
   {
      return minDistance;
   }

   public final DoubleProperty maxDistanceProperty()
   {
      return maxDistance;
   }

   public final DoubleProperty zoomSpeedFactorProperty()
   {
      return distanceModifier;
   }
}
