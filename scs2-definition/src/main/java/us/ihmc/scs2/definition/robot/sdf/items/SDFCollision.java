package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;

public class SDFCollision implements SDFItem
{
   private String name;
   private String pose;
   private Surface surface;
   private SDFGeometry geometry;

   public String getPose()
   {
      return pose;
   }

   @XmlElement(name = "pose")
   public void setPose(String pose)
   {
      this.pose = pose;
   }

   public String getName()
   {
      return name;
   }

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   public Surface getSurface()
   {
      return surface;
   }

   @XmlElement(name = "surface")
   public void setSurface(Surface surface)
   {
      this.surface = surface;
   }

   public SDFGeometry getGeometry()
   {
      return geometry;
   }

   @XmlElement(name = "geometry")
   public void setGeometry(SDFGeometry geometry)
   {
      this.geometry = geometry;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, pose: %s, surface: %s, geometry: %s]", name, pose, surface, geometry);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return SDFItem.combineItemURIHolders(surface, geometry);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   public static class Surface implements SDFItem
   {
      private Contact contact;

      public Contact getContact()
      {
         return contact;
      }

      @XmlElement(name = "contact")
      public void setContact(Contact contact)
      {
         this.contact = contact;
      }

      @Override
      public String getContentAsString()
      {
         return format("[contact: %s]", contact);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return SDFItem.combineItemURIHolders(contact);
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class Contact implements SDFItem
      {
         private Ode ode;

         public Ode getOde()
         {
            return ode;
         }

         @XmlElement(name = "ode")
         public void setOde(Ode ode)
         {
            this.ode = ode;
         }

         @Override
         public String getContentAsString()
         {
            return format("[ode: %s]", ode);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return SDFItem.combineItemURIHolders(ode);
         }

         @Override
         public String toString()
         {
            return itemToString();
         }

         public static class Ode implements SDFItem
         {
            private String kp;
            private String kd;
            private String maxVel;

            public String getKp()
            {
               return kp;
            }

            @XmlElement(name = "kp")
            public void setKp(String kp)
            {
               this.kp = kp;
            }

            public String getKd()
            {
               return kd;
            }

            @XmlElement(name = "kd")
            public void setKd(String kd)
            {
               this.kd = kd;
            }

            public String getMaxVel()
            {
               return maxVel;
            }

            @XmlElement(name = "max_vel")
            public void setMaxVel(String maxVel)
            {
               this.maxVel = maxVel;
            }

            @Override
            public String getContentAsString()
            {
               return format("[kp: %s, kd: %s, maxVel: %s]", kp, kd, maxVel);
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
}
