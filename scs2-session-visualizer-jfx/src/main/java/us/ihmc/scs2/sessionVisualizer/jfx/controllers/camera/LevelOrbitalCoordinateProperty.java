package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import static us.ihmc.scs2.definition.camera.YoLevelOrbitalCoordinateDefinition.YoLevelOrbital;
import static us.ihmc.scs2.definition.camera.YoLevelOrbitalCoordinateDefinition.YoLevelOrbitalIdentifiers;

import java.util.Objects;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.CompositeProperty;

public class LevelOrbitalCoordinateProperty extends CompositeProperty
{
   public LevelOrbitalCoordinateProperty()
   {
      super(YoLevelOrbital, YoLevelOrbitalIdentifiers);
   }

   public LevelOrbitalCoordinateProperty(double distance, double longitude, double height)
   {
      super(YoLevelOrbital, YoLevelOrbitalIdentifiers, distance, longitude, height);
   }

   public LevelOrbitalCoordinateProperty(ReferenceFrame referenceFrame, double distance, double longitude, double height)
   {
      super(YoLevelOrbital, YoLevelOrbitalIdentifiers, referenceFrame, distance, longitude, height);
   }

   public LevelOrbitalCoordinateProperty(DoubleProperty[] distanceLongitudeHeightProperties)
   {
      super(YoLevelOrbital, YoLevelOrbitalIdentifiers, distanceLongitudeHeightProperties);
   }

   public LevelOrbitalCoordinateProperty(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty[] distanceLongitudeHeightProperties)
   {
      super(YoLevelOrbital, YoLevelOrbitalIdentifiers, referenceFrameProperty, distanceLongitudeHeightProperties);
   }

   public LevelOrbitalCoordinateProperty(DoubleProperty distanceProperty, DoubleProperty longitudeProperty, DoubleProperty heightProperty)
   {
      super(YoLevelOrbital, YoLevelOrbitalIdentifiers, distanceProperty, longitudeProperty, heightProperty);
   }

   public LevelOrbitalCoordinateProperty(Property<ReferenceFrame> referenceFrameProperty,
                                               DoubleProperty distanceProperty,
                                               DoubleProperty longitudeProperty,
                                               DoubleProperty heightProperty)
   {
      super(YoLevelOrbital, YoLevelOrbitalIdentifiers, referenceFrameProperty, distanceProperty, longitudeProperty, heightProperty);
   }

   public LevelOrbitalCoordinateProperty(CompositeProperty other)
   {
      this();
      set(other);
   }

   public void setDistance(double distance)
   {
      setDistanceProperty(new SimpleDoubleProperty(distance));
   }

   public void setDistanceProperty(DoubleProperty distanceProperty)
   {
      componentValueProperties[0] = distanceProperty;
   }

   public void setLongitude(double longitude)
   {
      setLongitudeProperty(new SimpleDoubleProperty(longitude));
   }

   public void setLongitudeProperty(DoubleProperty longitudeProperty)
   {
      componentValueProperties[1] = longitudeProperty;
   }

   public void setHeight(double height)
   {
      setHeightProperty(new SimpleDoubleProperty(height));
   }

   public void setHeightProperty(DoubleProperty heightProperty)
   {
      componentValueProperties[2] = heightProperty;
   }

   public void set(double distance, double longitude, double height)
   {
      setComponentValues(distance, longitude, height);
   }

   public void set(ReferenceFrame referenceFrame, double distance, double longitude, double height)
   {
      set(referenceFrame, distance, longitude, height);
   }

   public void set(DoubleProperty distanceProperty, DoubleProperty longitudeProperty, DoubleProperty heightProperty)
   {
      setComponentValueProperties(distanceProperty, longitudeProperty, heightProperty);
   }

   public void set(Property<ReferenceFrame> referenceFrameProperty,
                   DoubleProperty distanceProperty,
                   DoubleProperty longitudeProperty,
                   DoubleProperty heightProperty)
   {
      set(referenceFrameProperty, distanceProperty, longitudeProperty, heightProperty);
   }

   @Override
   public void set(CompositeProperty other)
   {
      Objects.requireNonNull(other.getType());
      if (!getType().equals(other.getType()))
         throw new IllegalArgumentException("Cannot set a " + getClass().getSimpleName() + " to a " + other.getType());
      super.set(other);
   }

   public void set(LevelOrbitalCoordinateProperty other)
   {
      set((CompositeProperty) other);
   }

   public double getDistance()
   {
      return distanceProperty().get();
   }

   public DoubleProperty distanceProperty()
   {
      return componentValueProperties == null ? null : componentValueProperties[0];
   }

   public double getLongitude()
   {
      return longitudeProperty().get();
   }

   public DoubleProperty longitudeProperty()
   {
      return componentValueProperties == null ? null : componentValueProperties[1];
   }

   public double getHeight()
   {
      return heightProperty().get();
   }

   public DoubleProperty heightProperty()
   {
      return componentValueProperties == null ? null : componentValueProperties[2];
   }

   @Override
   public LevelOrbitalCoordinateProperty clone()
   {
      return new LevelOrbitalCoordinateProperty(this);
   }
}