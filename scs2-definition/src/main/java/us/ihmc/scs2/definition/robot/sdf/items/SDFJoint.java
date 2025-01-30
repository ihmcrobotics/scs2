package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;

public class SDFJoint implements SDFItem
{
   private String name;
   private String type;

   private String child;
   private String parent;
   private String pose;

   private String threadPitch;

   private SDFAxis axis;
   private SDFAxis axis2;

   @XmlAttribute(name = "name")
   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   @XmlAttribute(name = "type")
   public String getType()
   {
      return type;
   }

   public void setType(String type)
   {
      this.type = type;
   }

   public String getParent()
   {
      return parent;
   }

   @XmlElement(name = "parent")
   public void setParent(String parent)
   {
      this.parent = parent;
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

   public String getThreadPitch()
   {
      return threadPitch;
   }

   @XmlElement(name = "thread_pitch")
   public void setThreadPitch(String threadPitch)
   {
      this.threadPitch = threadPitch;
   }

   public SDFAxis getAxis()
   {
      return axis;
   }

   @XmlElement(name = "axis")
   public void setAxis(SDFAxis axis)
   {
      this.axis = axis;
   }

   public SDFAxis getAxis2()
   {
      return axis2;
   }

   @XmlElement(name = "axis2")
   public void setAxis2(SDFAxis axis2)
   {
      this.axis2 = axis2;
   }

   public String getChild()
   {
      return child;
   }

   @XmlElement(name = "child")
   public void setChild(String child)
   {
      this.child = child;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, type: %s, child: %s, parent: %s, pose: %s, threadPitch: %s, axis: %s, axis2: %s]",
                    name,
                    type,
                    child,
                    parent,
                    pose,
                    threadPitch,
                    axis,
                    axis2);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return SDFItem.combineItemURIHolders(axis, axis2);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   public static class SDFAxis implements SDFItem
   {
      private String xyz;

      private SDFDynamics dynamics;
      private SDFLimit limit;

      public String getXYZ()
      {
         return xyz;
      }

      @XmlElement(name = "xyz")
      public void setXYZ(String xyz)
      {
         this.xyz = xyz;
      }

      public SDFDynamics getDynamics()
      {
         return dynamics;
      }

      @XmlElement(name = "dynamics")
      public void setDynamics(SDFDynamics dynamics)
      {
         this.dynamics = dynamics;
      }

      public SDFLimit getLimit()
      {
         return limit;
      }

      @XmlElement(name = "limit")
      public void setLimit(SDFLimit limit)
      {
         this.limit = limit;
      }

      @Override
      public String getContentAsString()
      {
         return format("[xyz: %s, dynamics: %s, limit]", xyz, dynamics, limit);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return SDFItem.combineItemURIHolders(dynamics, limit);
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class SDFDynamics implements SDFItem
      {
         private String damping;
         private String friction;

         public String getDamping()
         {
            return damping;
         }

         @XmlElement(name = "damping")
         public void setDamping(String damping)
         {
            this.damping = damping;
         }

         public String getFriction()
         {
            return friction;
         }

         @XmlElement(name = "friction")
         public void setFriction(String friction)
         {
            this.friction = friction;
         }

         @Override
         public String getContentAsString()
         {
            return format("[damping: %s, friction: %s]", damping, friction);
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

      public static class SDFLimit implements SDFItem
      {
         private String lower;
         private String upper;

         private String effort;
         private String velocity;

         public String getLower()
         {
            return lower;
         }

         @XmlElement(name = "lower")
         public void setLower(String lower)
         {
            this.lower = lower;
         }

         public String getUpper()
         {
            return upper;
         }

         @XmlElement(name = "upper")
         public void setUpper(String upper)
         {
            this.upper = upper;
         }

         public String getEffort()
         {
            return effort;
         }

         @XmlElement(name = "effort")
         public void setEffort(String effort)
         {
            this.effort = effort;
         }

         public String getVelocity()
         {
            return velocity;
         }

         @XmlElement(name = "velocity")
         public void setVelocity(String velocity)
         {
            this.velocity = velocity;
         }

         @Override
         public String getContentAsString()
         {
            return format("[lower: %s, upper: %s, effort: %s, velocity: %s]", lower, upper, effort, velocity);
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
}