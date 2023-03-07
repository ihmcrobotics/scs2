package us.ihmc.scs2.definition.yoGraphic;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

/**
 * A {@code YoGraphicListDefinition} is convenience class that allows to gather several
 * {@code YoGraphicDefinition}s into a list itself being a {@link YoGraphicDefinition}. This permits
 * using the regular API for single yoGraphic.
 * 
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoGraphicList")
public class YoGraphicListDefinition extends YoGraphicDefinition
{
   /** The internal list of yoGraphics. */
   private List<YoGraphicDefinition> yoGraphics = new ArrayList<>();

   /**
    * Creates a new empty list.
    */
   public YoGraphicListDefinition()
   {
      registerListField("yoGraphics", this::getYoGraphics, this::setYoGraphics, "g", YoGraphicDefinition::toParsableString, YoGraphicDefinition::parse);
   }

   /**
    * Creates and initializes a new list.
    * 
    * @param yoGraphics the initial yoGraphics to include.
    */
   public YoGraphicListDefinition(YoGraphicDefinition... yoGraphics)
   {
      this(Arrays.asList(yoGraphics));
   }

   /**
    * Creates and initializes a new list.
    * 
    * @param yoGraphics the initial yoGraphics to include.
    */
   public YoGraphicListDefinition(Collection<? extends YoGraphicDefinition> yoGraphics)
   {
      setYoGraphics(new ArrayList<>(yoGraphics));
   }

   /**
    * Clears the list.
    */
   public void clear()
   {
      if (yoGraphics != null)
         yoGraphics.clear();
   }

   /**
    * Adds a yoGraphic to this list.
    * 
    * @param yoGraphic the yoGraphic to add.
    */
   public void addYoGraphic(YoGraphicDefinition yoGraphic)
   {
      if (yoGraphic == null)
         return;
      if (yoGraphic instanceof YoGraphicListDefinition list)
      {
         if (list.getYoGraphics() == null)
            return;
         for (int i = 0; i < list.getYoGraphics().size(); i++)
         {
            addYoGraphic(list.getYoGraphics().get(i));
         }
         return;
      }
      if (yoGraphics == null)
         yoGraphics = new ArrayList<>();
      yoGraphics.add(yoGraphic);
   }

   /**
    * Sets the list.
    * 
    * @param yoGraphics the new list.
    */
   @XmlElement(name = "yoGraphic")
   public void setYoGraphics(List<YoGraphicDefinition> yoGraphics)
   {
      this.yoGraphics = yoGraphics;
   }

   /**
    * Unwraps lists that have been added as elements to {@code this}.
    */
   public void unwrapNestedLists()
   {
      if (yoGraphics == null)
         return;

      for (int i = yoGraphics.size() - 1; i >= 0; i--)
      {
         YoGraphicDefinition yoGraphic = yoGraphics.get(i);

         if (yoGraphic instanceof YoGraphicListDefinition list)
         {
            yoGraphics.remove(i);
            if (list.getYoGraphics() != null)
            {
               list.unwrapNestedLists();
               yoGraphics.addAll(i, list.getYoGraphics());
            }
         }
      }
   }

   public List<YoGraphicDefinition> getYoGraphics()
   {
      return yoGraphics;
   }

   @Override
   public String toString(int indent)
   {
      String out = "%s [name=%s, visible=%b, yoGraphics=%s]";
      return out.formatted(getClass().getSimpleName(),
                           name,
                           visible,
                           indentedListString(indent, true, yoGraphics, yoGraphic -> yoGraphic.toString(indent + 1)));
   }
}
