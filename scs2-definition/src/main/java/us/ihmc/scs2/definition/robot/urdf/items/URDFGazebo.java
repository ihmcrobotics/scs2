package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.ArrayList;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;

public class URDFGazebo implements URDFItem
{
   private String reference;
   private String mu1;
   private String mu2;
   private String implicitSpringDamper;
   private String provideFeedback;
   private URDFSensor sensor;

   @XmlAttribute(name = "reference")
   public void setReference(String reference)
   {
      this.reference = reference;
   }

   @XmlElement(name = "mu1")
   public void setMu1(String mu1)
   {
      this.mu1 = mu1;
   }

   @XmlElement(name = "mu2")
   public void setMu2(String mu2)
   {
      this.mu2 = mu2;
   }

   @XmlElement(name = "implicitSpringDamper")
   public void setImplicitSpringDamper(String implicitSpringDamper)
   {
      this.implicitSpringDamper = implicitSpringDamper;
   }

   @XmlElement(name = "provideFeedback")
   public void setProvideFeedback(String provideFeedback)
   {
      this.provideFeedback = provideFeedback;
   }

   @XmlElement(name = "sensor")
   public void setSensor(URDFSensor sensor)
   {
      this.sensor = sensor;
   }

   public String getReference()
   {
      return reference;
   }

   public String getMu1()
   {
      return mu1;
   }

   public String getMu2()
   {
      return mu2;
   }

   public String getImplicitSpringDamper()
   {
      return implicitSpringDamper;
   }

   public String getProvideFeedback()
   {
      return provideFeedback;
   }

   public URDFSensor getSensor()
   {
      return sensor;
   }

   @Override
   public String getContentAsString()
   {
      String format = "[reference: %s";
      List<Object> items = new ArrayList<>();
      items.add(reference);
      if (mu1 != null)
      {
         format += ", mu1: %s";
         items.add(mu1);
      }
      if (mu2 != null)
      {
         format += ", mu2: %s";
         items.add(mu2);
      }
      if (implicitSpringDamper != null)
      {
         format += ", implicitSpringDamper: %s";
         items.add(implicitSpringDamper);
      }
      if (provideFeedback != null)
      {
         format += ", provideFeedback: %s";
         items.add(provideFeedback);
      }
      if (sensor != null)
      {
         format += ", sensor: %s";
         items.add(sensor);
      }
      return format(format, items.toArray());
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return URDFItem.combineItemFilenameHolders(sensor);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }
}
