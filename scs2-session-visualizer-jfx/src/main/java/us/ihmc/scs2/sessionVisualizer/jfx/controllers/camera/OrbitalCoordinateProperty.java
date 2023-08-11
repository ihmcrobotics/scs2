package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import static us.ihmc.scs2.definition.camera.YoOrbitalCoordinateDefinition.YoOrbital;
import static us.ihmc.scs2.definition.camera.YoOrbitalCoordinateDefinition.YoOrbitalIdentifiers;

import java.util.Objects;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.CompositeProperty;

public class OrbitalCoordinateProperty extends CompositeProperty
{
   public OrbitalCoordinateProperty()
   {
      super(YoOrbital, YoOrbitalIdentifiers);
   }

   public OrbitalCoordinateProperty(double distance, double longitude, double latitude)
   {
      super(YoOrbital, YoOrbitalIdentifiers, distance, longitude, latitude);
   }

   public OrbitalCoordinateProperty(ReferenceFrame referenceFrame, double distance, double longitude, double latitude)
   {
      super(YoOrbital, YoOrbitalIdentifiers, referenceFrame, distance, longitude, latitude);
   }

   public OrbitalCoordinateProperty(DoubleProperty[] distanceLongitudeLatitudeProperties)
   {
      super(YoOrbital, YoOrbitalIdentifiers, distanceLongitudeLatitudeProperties);
   }

   public OrbitalCoordinateProperty(Property<ReferenceFrame> referenceFrameProperty, DoubleProperty[] distanceLongitudeLatitudeProperties)
   {
      super(YoOrbital, YoOrbitalIdentifiers, referenceFrameProperty, distanceLongitudeLatitudeProperties);
   }

   public OrbitalCoordinateProperty(DoubleProperty distanceProperty, DoubleProperty longitudeProperty, DoubleProperty latitudeProperty)
   {
      super(YoOrbital, YoOrbitalIdentifiers, distanceProperty, longitudeProperty, latitudeProperty);
   }

   public OrbitalCoordinateProperty(Property<ReferenceFrame> referenceFrameProperty,
                                          DoubleProperty distanceProperty,
                                          DoubleProperty longitudeProperty,
                                          DoubleProperty latitudeProperty)
   {
      super(YoOrbital, YoOrbitalIdentifiers, referenceFrameProperty, distanceProperty, longitudeProperty, latitudeProperty);
   }

   public OrbitalCoordinateProperty(CompositeProperty other)
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

   public void setLatitude(double latitude)
   {
      setLatitudeProperty(new SimpleDoubleProperty(latitude));
   }

   public void setLatitudeProperty(DoubleProperty latitudeProperty)
   {
      componentValueProperties[2] = latitudeProperty;
   }

   public void set(double distance, double longitude, double latitude)
   {
      setComponentValues(distance, longitude, latitude);
   }

   public void set(ReferenceFrame referenceFrame, double distance, double longitude, double latitude)
   {
      set(referenceFrame, distance, longitude, latitude);
   }

   public void set(DoubleProperty distanceProperty, DoubleProperty longitudeProperty, DoubleProperty latitudeProperty)
   {
      setComponentValueProperties(distanceProperty, longitudeProperty, latitudeProperty);
   }

   public void set(Property<ReferenceFrame> referenceFrameProperty,
                   DoubleProperty distanceProperty,
                   DoubleProperty longitudeProperty,
                   DoubleProperty latitudeProperty)
   {
      set(referenceFrameProperty, distanceProperty, longitudeProperty, latitudeProperty);
   }

   @Override
   public void set(CompositeProperty other)
   {
      Objects.requireNonNull(other.getType());
      if (!getType().equals(other.getType()))
         throw new IllegalArgumentException("Cannot set a " + getClass().getSimpleName() + " to a " + other.getType());
      super.set(other);
   }

   public void set(OrbitalCoordinateProperty other)
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

   public double getLatitude()
   {
      return latitudeProperty().get();
   }

   public DoubleProperty latitudeProperty()
   {
      return componentValueProperties == null ? null : componentValueProperties[2];
   }

   @Override
   public OrbitalCoordinateProperty clone()
   {
      return new OrbitalCoordinateProperty(this);
   }
}