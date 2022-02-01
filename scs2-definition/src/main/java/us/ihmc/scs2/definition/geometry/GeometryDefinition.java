package us.ihmc.scs2.definition.geometry;

import java.util.Objects;

import javax.xml.bind.annotation.XmlAttribute;

public abstract class GeometryDefinition
{
   private String name;

   /**
    * Sets the name to associate with this definition. The name can be passed to objects or graphics
    * created using this definition.
    *
    * @param name the geometry's name.
    */
   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   /**
    * Returns this geometry's name.
    *
    * @return the name associated with this definition.
    */
   public String getName()
   {
      return name;
   }

   public abstract GeometryDefinition copy();

   @Override
   public int hashCode()
   {
      return Objects.hashCode(name);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      GeometryDefinition other = (GeometryDefinition) object;
      if (!Objects.equals(name, other.name))
         return false;
      return true;
   }
}
