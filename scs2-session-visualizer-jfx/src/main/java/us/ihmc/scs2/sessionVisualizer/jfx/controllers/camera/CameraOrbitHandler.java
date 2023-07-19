package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import java.util.function.Predicate;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.event.EventHandler;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.TransformChangedEvent;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

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
    * The current orientation of the camera. This is the output of this calculator which can be bound
    * to an external property or used directly to apply a transformation to the camera.
    */
   private final Affine cameraOrientation = new Affine();
   /**
    * Rotation offset computed such that when the {@link #latitude}, {@link #longitude}, and
    * {@link #roll} are all zero, the camera is orientated as follows:
    * <li>the viewing direction (e.g. z-axis) of the camera is aligned with the given forward axis.
    * <li>the vertical direction of the screen (e.g. y-axis) is collinear with the given up axis.
    */
   private final Affine offset = new Affine();
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
   /**
    * Zoom speed factor with respect to its current value. The larger is the zoom, the faster it
    * "goes".
    */
   private final DoubleProperty distanceModifier = new SimpleDoubleProperty(this, "distanceModifier", -0.1);

   private final Vector3D up = new Vector3D();
   private final Vector3D down = new Vector3D();
   private final Vector3D forward = new Vector3D();

   private boolean disableCameraOrientationAutoUpdate = false;

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

      this.up.set(up);
      this.forward.set(forward);
      down.setAndNegate(up);

      computeOffset();

      ChangeListener<? super Number> listener = (o, oldValue, newValue) ->
      {
         if (!disableCameraOrientationAutoUpdate)
            updateCameraOrientation(false);
      };
      latitude.addListener(listener);
      longitude.addListener(listener);
      roll.addListener(listener);

      cameraOrientation.addEventHandler(TransformChangedEvent.TRANSFORM_CHANGED, e -> updateCameraPose());
      distance.addListener((o, oldValue, newValue) -> updateCameraPose());
   }

   private void computeOffset()
   {
      Vector3D cameraZAxis = new Vector3D(forward);
      Vector3D cameraYAxis = new Vector3D(down);
      Vector3D cameraXAxis = new Vector3D();
      cameraXAxis.cross(cameraYAxis, cameraZAxis);
      RotationMatrix rotationOffset = new RotationMatrix();
      rotationOffset.setColumns(cameraXAxis, cameraYAxis, cameraZAxis);
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
      return new EventHandler<MouseEvent>()
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

            Vector2D centerToMouseLocation = new Vector2D();
            centerToMouseLocation.sub(newMouseLocation, centerLocation);
            double rollShift = 0.0 * modifier * rollModifier.get() * drag.cross(centerToMouseLocation);

            drag.scale(modifier);
            rotate(drag.getY(), -drag.getX(), rollShift);

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
      return new EventHandler<ScrollEvent>()
      {
         @Override
         public void handle(ScrollEvent event)
         {
            double direction = Math.signum(event.getDeltaY());
            double newDistance = distance.get() + direction * distance.get() * distanceModifier.get();
            newDistance = MathTools.clamp(newDistance, minDistance.get(), maxDistance.get());
            distance.set(newDistance);
         }
      };
   }

   /**
    * Update the camera rotation after applying rotation offsets.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param deltaLatitude  the shift in latitude to apply to the camera rotation.
    * @param deltaLongitude the shift in longitude to apply to the camera rotation.
    * @param deltaRoll      the shift in roll to apply to the camera rotation.
    */
   public void rotate(double deltaLatitude, double deltaLongitude, double deltaRoll)
   {
      rotate(deltaLatitude, deltaLongitude, deltaRoll, false);
   }

   /**
    * Update the camera rotation after applying rotation offsets.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param deltaLatitude  the shift in latitude to apply to the camera rotation.
    * @param deltaLongitude the shift in longitude to apply to the camera rotation.
    * @param deltaRoll      the shift in roll to apply to the camera rotation.
    * @return the amount to translate the focal point to use for making the camera rotate on itself
    *         instead of orbiting around a fixed focal point.
    */
   public Vector3D rotate(double deltaLatitude, double deltaLongitude, double deltaRoll, boolean computeFocalPointShift)
   {
      disableCameraOrientationAutoUpdate = true;
      if (Double.isFinite(deltaLatitude))
      {
         double newLatitude = latitude.get() + deltaLatitude;
         if (restrictLatitude.get())
            newLatitude = MathTools.clamp(newLatitude, minLatitude.get(), maxLatitude.get());
         else
            newLatitude = MathTools.clamp(newLatitude, Math.PI);
         latitude.set(newLatitude);
      }

      if (Double.isFinite(deltaLongitude))
      {
         double newLongitude = longitude.get() + deltaLongitude;
         newLongitude = EuclidCoreTools.trimAngleMinusPiToPi(newLongitude);
         longitude.set(newLongitude);
      }

      if (keepRotationLeveled.get())
      {
         roll.set(0.0);
      }
      else if (Double.isFinite(deltaRoll))
      {
         double newRoll = roll.get() + deltaRoll;
         newRoll = EuclidCoreTools.trimAngleMinusPiToPi(newRoll);
         roll.set(newRoll);
      }
      Vector3D focalPointTranslation = updateCameraOrientation(computeFocalPointShift);
      disableCameraOrientationAutoUpdate = false;
      return focalPointTranslation;
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
      disableCameraOrientationAutoUpdate = true;
      Vector3D fromCameraToFocus = new Vector3D();
      fromCameraToFocus.setX(Double.isFinite(x) ? -x : cameraPose.getTx());
      fromCameraToFocus.setY(Double.isFinite(y) ? -y : cameraPose.getTy());
      fromCameraToFocus.setZ(Double.isFinite(z) ? -z : cameraPose.getTz());
      distance.set(fromCameraToFocus.norm());
      fromCameraToFocus.scale(1.0 / distance.get());

      double newLatitude = fromCameraToFocus.angle(up) - Math.PI / 2.0;
      // We remove the component along up to be able to compute the longitude
      fromCameraToFocus.scaleAdd(-fromCameraToFocus.dot(up), up, fromCameraToFocus);

      double newLongitude = fromCameraToFocus.angle(forward);

      Vector3D cross = new Vector3D();
      cross.cross(fromCameraToFocus, forward);

      if (cross.dot(up) > 0.0)
         newLongitude = -newLongitude;

      latitude.set(newLatitude);
      longitude.set(newLongitude);
      this.roll.set(roll);
      updateCameraOrientation(false);
      disableCameraOrientationAutoUpdate = false;
   }

   /**
    * Sets the camera's latitude, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param latitude  the new camera latitude.
    * @param longitude the new camera longitude.
    * @param roll      the new camera roll.
    */
   public void setRotation(double latitude, double longitude, double roll)
   {
      setRotation(latitude, longitude, roll, false);
   }

   /**
    * Sets the camera's latitude, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param latitude  the new camera latitude.
    * @param longitude the new camera longitude.
    * @param roll      the new camera roll.
    * @return the amount to translate the focal point to use for making the camera rotate on itself
    *         instead of orbiting around a fixed focal point.
    */
   public Vector3D setRotation(double latitude, double longitude, double roll, boolean computeFocalPointShift)
   {
      disableCameraOrientationAutoUpdate = true;
      if (Double.isFinite(latitude))
      {
         if (restrictLatitude.get())
            this.latitude.set(MathTools.clamp(latitude, minLatitude.get(), maxLatitude.get()));
         else
            this.latitude.set(MathTools.clamp(latitude, Math.PI / 2.0));
      }

      if (Double.isFinite(longitude))
         this.longitude.set(EuclidCoreTools.trimAngleMinusPiToPi(longitude));

      if (Double.isFinite(roll))
         this.roll.set(EuclidCoreTools.trimAngleMinusPiToPi(roll));

      Vector3D focalPointTranslation = updateCameraOrientation(computeFocalPointShift);
      disableCameraOrientationAutoUpdate = false;
      return focalPointTranslation;
   }

   /**
    * Sets the camera's distance from focal point, latitude, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param distance  the new distance between camera and focal point.
    * @param latitude  the new camera latitude.
    * @param longitude the new camera longitude.
    * @param roll      the new camera roll.
    */
   public void setOrbit(double distance, double latitude, double longitude, double roll)
   {
      setOrbit(distance, latitude, longitude, roll, false);
   }

   /**
    * Sets the camera's distance from focal point, latitude, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param distance  the new distance between camera and focal point.
    * @param latitude  the new camera latitude.
    * @param longitude the new camera longitude.
    * @param roll      the new camera roll.
    * @return the amount to translate the focal point to use for making the camera rotate on itself
    *         instead of orbiting around a fixed focal point.
    */
   public Vector3D setOrbit(double distance, double latitude, double longitude, double roll, boolean computeFocalPointShift)
   {
      if (Double.isFinite(distance))
         this.distance.set(distance);
      return setRotation(latitude, longitude, roll, computeFocalPointShift);
   }

   /**
    * Sets the camera's distance from focal point, height, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param distance  the new distance between camera and focal point.
    * @param height    the new camera height.
    * @param longitude the new camera longitude.
    * @param roll      the new camera roll.
    */
   public void setLevelOrbit(double distance, double height, double longitude, double roll)
   {
      setLevelOrbit(distance, height, longitude, roll, false);
   }

   /**
    * Sets the camera's distance from focal point, height, longitude, and roll.
    * <p>
    * Non-finite values are ignored.
    * </p>
    *
    * @param distance  the new distance between camera and focal point.
    * @param height    the new camera height.
    * @param longitude the new camera longitude.
    * @param roll      the new camera roll.
    * @return the amount to translate the focal point to use for making the camera rotate on itself
    *         instead of orbiting around a fixed focal point.
    */
   public Vector3D setLevelOrbit(double distance, double height, double longitude, double roll, boolean computeFocalPointShift)
   {
      if (Double.isFinite(distance))
         this.distance.set(distance);
      disableCameraOrientationAutoUpdate = true;

      if (Double.isFinite(height))
         latitude.set(Math.asin(height / this.distance.get()));

      if (Double.isFinite(longitude))
         this.longitude.set(EuclidCoreTools.trimAngleMinusPiToPi(longitude));

      if (Double.isFinite(roll))
         this.roll.set(EuclidCoreTools.trimAngleMinusPiToPi(roll));

      Vector3D focalPointTranslation = updateCameraOrientation(computeFocalPointShift);
      disableCameraOrientationAutoUpdate = false;
      return focalPointTranslation;
   }

   private Vector3D updateCameraOrientation(boolean computeFocalPointShift)
   {
      Vector3D focalPointTranslation = null;

      if (computeFocalPointShift)
      {
         focalPointTranslation = new Vector3D(cameraPose.getTx(), cameraPose.getTy(), cameraPose.getTz());
      }

      Affine newRotation = new Affine();
      newRotation.append(offset);
      newRotation.append(new Rotate(Math.toDegrees(-longitude.get()), Rotate.Y_AXIS));
      newRotation.append(new Rotate(Math.toDegrees(-latitude.get()), Rotate.X_AXIS));
      newRotation.append(new Rotate(Math.toDegrees(roll.get()), Rotate.Z_AXIS));
      cameraOrientation.setToTransform(newRotation);

      if (computeFocalPointShift)
      {
         focalPointTranslation.add(distance.get() * cameraOrientation.getMxz(),
                                   distance.get() * cameraOrientation.getMyz(),
                                   distance.get() * cameraOrientation.getMzz());
         return focalPointTranslation;
      }
      return null;
   }

   private void updateCameraPose()
   {
      cameraPose.setToTransform(cameraOrientation);
      cameraPose.appendTranslation(0.0, 0.0, -distance.get()); // we need to shift the camera backward
   }

   public Affine getCameraPose()
   {
      return cameraPose;
   }

   /**
    * Get the reference to the rotation of the camera. This is the output of this calculator which can
    * be bound to an external property or used directly to apply a transformation to the camera.
    *
    * @return the camera's rotation.
    */
   public Affine getCameraRotation()
   {
      return cameraOrientation;
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

   public static interface PositionCoodinateProvider
   {
      double getX();

      double getY();

      double getZ();
   }

   public static interface OrbitalCoordinateProvider
   {
      double getDistance();

      double getLatitude();

      double getLongitude();

      double getRoll();
   }

   public static interface LevelOrbitalCoordinateProvider
   {
      double getDistance();

      double getLongitude();

      double getHeight();

      double getRoll();
   }
}
