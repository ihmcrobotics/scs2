package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFLink implements SDFItem
{
   private String name;
   private String pose;
   private SDFInertial inertial;
   private List<SDFVisual> visuals;
   private List<SDFSensor> sensors;
   private List<SDFCollision> collisions;

   public String getName()
   {
      return name;
   }

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   public String getPose()
   {
      return pose;
   }

   @XmlElement(name = "pose")
   public void setPose(String pose)
   {
      this.pose = pose;
   }

   public SDFInertial getInertial()
   {
      return inertial;
   }

   @XmlElement(name = "inertial")
   public void setInertial(SDFInertial inertial)
   {
      this.inertial = inertial;
   }

   public List<SDFVisual> getVisuals()
   {
      return visuals;
   }

   @XmlElement(name = "visual")
   public void setVisuals(List<SDFVisual> visual)
   {
      visuals = visual;
   }

   public List<SDFCollision> getCollisions()
   {
      return collisions;
   }

   @XmlElement(name = "collision")
   public void setCollisions(List<SDFCollision> collision)
   {
      collisions = collision;
   }

   public List<SDFSensor> getSensors()
   {
      return sensors;
   }

   @XmlElement(name = "sensor")
   public void setSensors(List<SDFSensor> sensors)
   {
      this.sensors = sensors;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, pose: %s, inertial: %s, visuals: %s, sensors: %s, collisions: %s]", name, pose, inertial, visuals, sensors, collisions);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return SDFItem.combineItemListsURIHolders(Arrays.asList(inertial), visuals, sensors, collisions);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   public static class SDFInertial implements SDFItem
   {
      private String mass;
      private String pose;
      private SDFInertia inertia;

      public String getMass()
      {
         return mass;
      }

      @XmlElement(name = "mass")
      public void setMass(String mass)
      {
         this.mass = mass;
      }

      public String getPose()
      {
         return pose;
      }

      @XmlElement(name = "pose")
      public void setPose(String pose)
      {
         this.pose = pose;
      }

      public SDFInertia getInertia()
      {
         return inertia;
      }

      @XmlElement(name = "inertia")
      public void setInertia(SDFInertia inertia)
      {
         this.inertia = inertia;
      }

      @Override
      public String getContentAsString()
      {
         return format("[mass: %s, pose: %s, inertia: %s]", mass, pose, inertia);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }
   }
}