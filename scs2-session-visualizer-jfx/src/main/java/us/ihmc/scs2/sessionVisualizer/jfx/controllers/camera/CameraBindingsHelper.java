package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

public class CameraBindingsHelper
{
   static void removeCameraPositionBindings(Tuple3DProperty coordinates, CameraOrbitHandler orbitHandler)
   {
      orbitHandler.xWorldProperty().unbind();
      orbitHandler.yWorldProperty().unbind();
      orbitHandler.zWorldProperty().unbind();

      if (coordinates == null)
         return;

      if (coordinates.xProperty() != null)
         orbitHandler.xWorldProperty().unbindBidirectional(coordinates.xProperty());
      if (coordinates.yProperty() != null)
         orbitHandler.yWorldProperty().unbindBidirectional(coordinates.yProperty());
      if (coordinates.zProperty() != null)
         orbitHandler.zWorldProperty().unbindBidirectional(coordinates.zProperty());
   }

   static void addCameraPositionBindings(Tuple3DProperty coordinates, CameraOrbitHandler orbitHandler)
   {
      if (coordinates == null)
         return;

      if (coordinates.xProperty() != null)
      {
         if (coordinates.xProperty() instanceof YoDoubleProperty)
            orbitHandler.xWorldProperty().bind(coordinates.xProperty());
         else
            orbitHandler.xWorldProperty().bindBidirectional(coordinates.xProperty());
      }

      if (coordinates.yProperty() != null)
      {
         if (coordinates.yProperty() instanceof YoDoubleProperty)
            orbitHandler.yWorldProperty().bind(coordinates.yProperty());
         else
            orbitHandler.yWorldProperty().bindBidirectional(coordinates.yProperty());
      }

      if (coordinates.zProperty() != null)
      {
         if (coordinates.zProperty() instanceof YoDoubleProperty)
            orbitHandler.zWorldProperty().bind(coordinates.zProperty());
         else
            orbitHandler.zWorldProperty().bindBidirectional(coordinates.zProperty());
      }
   }

   static void removeCameraOrbitalBindings(OrbitalCoordinateProperty coordinates, CameraOrbitHandler orbitHandler)
   {
      orbitHandler.distanceProperty().unbind();
      orbitHandler.longitudeProperty().unbind();
      orbitHandler.latitudeProperty().unbind();

      if (coordinates == null)
         return;

      if (coordinates.distanceProperty() != null)
         orbitHandler.distanceProperty().unbindBidirectional(coordinates.distanceProperty());
      if (coordinates.longitudeProperty() != null)
         orbitHandler.longitudeProperty().unbindBidirectional(coordinates.longitudeProperty());
      if (coordinates.latitudeProperty() != null)
         orbitHandler.latitudeProperty().unbindBidirectional(coordinates.latitudeProperty());
   }

   static void addCameraOrbitalBindings(OrbitalCoordinateProperty coordinates, CameraOrbitHandler orbitHandler)
   {
      if (coordinates == null)
         return;

      if (coordinates.distanceProperty() != null)
      {
         if (coordinates.distanceProperty() instanceof YoDoubleProperty)
            orbitHandler.distanceProperty().bind(coordinates.distanceProperty());
         else
            orbitHandler.distanceProperty().bindBidirectional(coordinates.distanceProperty());
      }

      if (coordinates.longitudeProperty() != null)
      {
         if (coordinates.longitudeProperty() instanceof YoDoubleProperty)
            orbitHandler.longitudeProperty().bind(coordinates.longitudeProperty());
         else
            orbitHandler.longitudeProperty().bindBidirectional(coordinates.longitudeProperty());
      }

      if (coordinates.latitudeProperty() != null)
      {
         if (coordinates.latitudeProperty() instanceof YoDoubleProperty)
            orbitHandler.latitudeProperty().bind(coordinates.latitudeProperty());
         else
            orbitHandler.latitudeProperty().bindBidirectional(coordinates.latitudeProperty());
      }
   }

   static void removeCameraLevelOrbitalBindings(LevelOrbitalCoordinateProperty coordinates, CameraOrbitHandler orbitHandler)
   {
      orbitHandler.distanceProperty().unbind();
      orbitHandler.longitudeProperty().unbind();
      orbitHandler.zWorldProperty().unbind();

      if (coordinates == null)
         return;

      if (coordinates.distanceProperty() != null)
         orbitHandler.distanceProperty().unbindBidirectional(coordinates.distanceProperty());
      if (coordinates.longitudeProperty() != null)
         orbitHandler.longitudeProperty().unbindBidirectional(coordinates.longitudeProperty());
      if (coordinates.heightProperty() != null)
         orbitHandler.zWorldProperty().unbindBidirectional(coordinates.heightProperty());
   }

   static void addCameraLevelOrbitalBindings(LevelOrbitalCoordinateProperty coordinates, CameraOrbitHandler orbitHandler)
   {
      if (coordinates == null)
         return;

      if (coordinates.distanceProperty() != null)
      {
         if (coordinates.distanceProperty() instanceof YoDoubleProperty)
            orbitHandler.distanceProperty().bind(coordinates.distanceProperty());
         else
            orbitHandler.distanceProperty().bindBidirectional(coordinates.distanceProperty());
      }

      if (coordinates.longitudeProperty() != null)
      {
         if (coordinates.longitudeProperty() instanceof YoDoubleProperty)
            orbitHandler.longitudeProperty().bind(coordinates.longitudeProperty());
         else
            orbitHandler.longitudeProperty().bindBidirectional(coordinates.longitudeProperty());
      }

      if (coordinates.heightProperty() != null)
      {
         if (coordinates.heightProperty() instanceof YoDoubleProperty)
            orbitHandler.zWorldProperty().bind(coordinates.heightProperty());
         else
            orbitHandler.zWorldProperty().bindBidirectional(coordinates.heightProperty());
      }
   }
}
