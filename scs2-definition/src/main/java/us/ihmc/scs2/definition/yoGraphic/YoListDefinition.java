package us.ihmc.scs2.definition.yoGraphic;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoList")
public class YoListDefinition
{
   private List<String> elements;
   private String size;

   public YoListDefinition()
   {
   }

   public YoListDefinition(List<String> elements)
   {
      this.elements = elements;
   }

   public YoListDefinition(List<String> elements, String size)
   {
      this.elements = elements;
      this.size = size;
   }

   @XmlElement
   public void setElements(List<String> elements)
   {
      this.elements = elements;
   }

   public void addElement(String element)
   {
      if (elements == null)
         elements = new ArrayList<>();
      elements.add(element);
   }

   public void addElement(double element)
   {
      addElement(Double.toString(element));
   }

   public void addElement(float element)
   {
      addElement(Float.toString(element));
   }

   public void addElement(boolean element)
   {
      addElement(Boolean.toString(element));
   }

   public void addElement(int element)
   {
      addElement(Integer.toString(element));
   }

   public void addElement(long element)
   {
      addElement(Long.toString(element));
   }

   public void addElement(byte element)
   {
      addElement(Byte.toString(element));
   }

   public void setElements(double[] elements)
   {
      for (double element : elements)
         addElement(element);
   }

   public void setElements(float[] elements)
   {
      for (float element : elements)
         addElement(element);
   }

   public void setElements(boolean[] elements)
   {
      for (boolean element : elements)
         addElement(element);
   }

   public void setElements(int[] elements)
   {
      for (int element : elements)
         addElement(element);
   }

   public void setElements(long[] elements)
   {
      for (long element : elements)
         addElement(element);
   }

   public void setElements(byte[] elements)
   {
      for (byte element : elements)
         addElement(element);
   }

   @XmlElement
   public void setSize(String size)
   {
      this.size = size;
   }

   public void setSize(int size)
   {
      this.size = Integer.toString(size);
   }

   public List<String> getElements()
   {
      return elements;
   }

   public String getSize()
   {
      return size;
   }

   @Override
   public int hashCode()
   {
      return Objects.hash(elements, size);
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

      YoListDefinition other = (YoListDefinition) object;

      if (!Objects.equals(elements, other.elements))
         return false;
      if (!Objects.equals(size, other.size))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "YoList(elements=" + elements + ", size=" + size + ")";
   }

   public static YoListDefinition parse(String value)
   {
      value = value.trim();

      if (value.startsWith("YoList"))
      {
         String[] elements;

         int elementsFirstIndex = value.indexOf("[");
         if (elementsFirstIndex == -1)
         {
            elements = null;
         }
         else
         {
            String elementsSubstring = value.substring(elementsFirstIndex + 1, value.indexOf("]")).trim();
            if (elementsSubstring.isEmpty())
            {
               elements = new String[0];
            }
            else
            {
               elements = value.substring(value.indexOf("=") + 2, value.lastIndexOf(",") - 1).split(",");
               for (int i = 0; i < elements.length; i++)
                  elements[i] = elements[i].trim();
            }
         }

         String size = value.substring(value.lastIndexOf("=") + 1, value.length() - 1).trim();

         if (size.equalsIgnoreCase("null"))
            size = null;

         return new YoListDefinition(elements == null ? null : Arrays.asList(elements), size);
      }
      else
      {
         throw new IllegalArgumentException("Unknown yoList format: " + value);
      }
   }
}
