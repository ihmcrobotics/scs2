package us.ihmc.scs2.definition.yoGraphic;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.*;

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
   /**
    * The internal list of yoGraphics.
    */
   private List<YoGraphicDefinition> yoGraphics = new ArrayList<>();

   /**
    * Creates a new empty list.
    */
   public YoGraphicListDefinition()
   {
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
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicListDefinition(YoGraphicListDefinition other)
   {
      super(other);
      if (other.yoGraphics != null)
         yoGraphics = other.yoGraphics.stream().filter(Objects::nonNull).map(YoGraphicDefinition::copy).toList();
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerListField("yoGraphics", this::getYoGraphics, this::setYoGraphics, "g", YoGraphicDefinition::toParsableString, YoGraphicDefinition::parse);
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

   /**
    * Merges any groups sharing the same name. The operation is propagated to any group and y list
    * registered.
    */
   public void mergeGroupsByName()
   {
      if (yoGraphics == null)
         return;

      for (int i = yoGraphics.size() - 1; i >= 0; i--)
      {
         YoGraphicDefinition yoGraphic = yoGraphics.get(i);
         if (yoGraphic instanceof YoGraphicGroupDefinition subGroup)
         {
            subGroup.mergeNestedGroupsByName();

            for (int j = yoGraphics.size() - 1; j > i; j--)
            {
               YoGraphicDefinition otherChild = yoGraphics.get(j);
               if (otherChild instanceof YoGraphicGroupDefinition otherSubGroup)
               {
                  if (subGroup.getName().equals(otherSubGroup.getName()))
                  {
                     yoGraphics.remove(j);
                     if (otherSubGroup.getChildren() != null)
                        subGroup.getChildren().addAll(otherSubGroup.getChildren());
                  }
               }
            }
         }
         else if (yoGraphic instanceof YoGraphicListDefinition list)
         {
            list.mergeGroupsByName();
         }
      }
   }

   @Override
   public YoGraphicListDefinition copy()
   {
      return new YoGraphicListDefinition(this);
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
