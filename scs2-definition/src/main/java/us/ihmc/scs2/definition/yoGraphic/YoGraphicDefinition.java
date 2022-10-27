package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlAttribute;

public abstract class YoGraphicDefinition
{
   protected String name;
   protected boolean visible = true;

   public YoGraphicDefinition()
   {
   }

   public YoGraphicDefinition(String name)
   {
      setName(name);
   }

   @XmlAttribute
   public final void setName(String name)
   {
      this.name = name;
   }

   @XmlAttribute
   public final void setVisible(boolean visible)
   {
      this.visible = visible;
   }

   public final String getName()
   {
      return name;
   }

   public final boolean isVisible()
   {
      return visible;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoGraphicDefinition)
      {
         YoGraphicDefinition other = (YoGraphicDefinition) object;

         if (name == null ? other.name != null : !name.equals(other.name))
            return false;
         if (visible != other.visible)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
