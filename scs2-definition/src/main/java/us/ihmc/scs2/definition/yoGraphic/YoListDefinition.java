package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * A {@code YoListDefinition} is a template to build a list which elements can be
 * {@code YoVariable}s and size can be controlled by a {@code YoVariable}.
 *
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoList")
public class YoListDefinition
{
   /** The elements in this list, can either be constants or {@code YoVariable}s. */
   private List<String> elements;
   /**
    * The size of the active part of the list, can be a constant, a {@code YoVariable}, or {@code null}
    * to consider all the elements.
    */
   private String size;

   /**
    * Creates a new empty list.
    */
   public YoListDefinition()
   {
   }

   /**
    * Creates a new list and initializes its elements.
    *
    * @param elements the initial list of elements.
    */
   public YoListDefinition(List<String> elements)
   {
      this.elements = elements;
   }

   /**
    * Creates a new list and initializes its elements and size variable.
    *
    * @param elements the initial list of elements.
    * @param size     the size of the active part of the list.
    */
   public YoListDefinition(List<String> elements, String size)
   {
      this.elements = elements;
      this.size = size;
   }

   /**
    * Sets the list elements.
    *
    * @param elements the elements.
    */
   @XmlElement
   public void setElements(List<String> elements)
   {
      this.elements = elements;
   }

   /**
    * Adds an element to the list, can be a constant or a {@code YoVariable} by giving its
    * name/fullname.
    *
    * @param element the new element.
    */
   public void addElement(String element)
   {
      if (elements == null)
         elements = new ArrayList<>();
      elements.add(element);
   }

   /**
    * Convenience method for adding an element representing a constant value.
    *
    * @param element the new element.
    */
   public void addElement(double element)
   {
      addElement(Double.toString(element));
   }

   /**
    * Convenience method for adding an element representing a constant value.
    *
    * @param element the new element.
    */
   public void addElement(float element)
   {
      addElement(Float.toString(element));
   }

   /**
    * Convenience method for adding an element representing a constant value.
    *
    * @param element the new element.
    */
   public void addElement(boolean element)
   {
      addElement(Boolean.toString(element));
   }

   /**
    * Convenience method for adding an element representing a constant value.
    *
    * @param element the new element.
    */
   public void addElement(int element)
   {
      addElement(Integer.toString(element));
   }

   /**
    * Convenience method for adding an element representing a constant value.
    *
    * @param element the new element.
    */
   public void addElement(long element)
   {
      addElement(Long.toString(element));
   }

   /**
    * Convenience method for adding an element representing a constant value.
    *
    * @param element the new element.
    */
   public void addElement(byte element)
   {
      addElement(Byte.toString(element));
   }

   public void setElements(double[] elements)
   {
      for (double element : elements)
         addElement(element);
   }

   /**
    * Convenience method for adding a elements representing constant values.
    *
    * @param elements the new elements.
    */
   public void setElements(float[] elements)
   {
      for (float element : elements)
         addElement(element);
   }

   /**
    * Convenience method for adding a elements representing constant values.
    *
    * @param elements the new elements.
    */
   public void setElements(boolean[] elements)
   {
      for (boolean element : elements)
         addElement(element);
   }

   /**
    * Convenience method for adding a elements representing constant values.
    *
    * @param elements the new elements.
    */
   public void setElements(int[] elements)
   {
      for (int element : elements)
         addElement(element);
   }

   /**
    * Convenience method for adding a elements representing constant values.
    *
    * @param elements the new elements.
    */
   public void setElements(long[] elements)
   {
      for (long element : elements)
         addElement(element);
   }

   /**
    * Convenience method for adding a elements representing constant values.
    *
    * @param elements the new elements.
    */
   public void setElements(byte[] elements)
   {
      for (byte element : elements)
         addElement(element);
   }

   /**
    * Sets the variable for tracking the active part of the list, can be {@code null} for using all
    * elements, a constant value, or a {@code YoVaribale} by giving its name/fullname.
    *
    * @param size the size of the active part of the list.
    */
   @XmlElement
   public void setSize(String size)
   {
      this.size = size;
   }

   /**
    * Sets the size of the list to a constant value.
    *
    * @param size the size of the active part of the list.
    */
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

   public YoListDefinition copy()
   {
      return new YoListDefinition(elements == null ? null : new ArrayList<>(elements), size);
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
