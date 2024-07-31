package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

/**
 * A {@link YoGraphicGroupDefinition} is a template for creating a group which contains yoGraphics
 * and sub-groups. This is useful for organizing yoGraphics in a similar way that
 * {@code YoVariable}s are organized with {@code YoRegistry}s.
 * <p>
 * The group visible property is propagated down its descendants.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoGraphicGroup")
public class YoGraphicGroupDefinition extends YoGraphicDefinition
{
   /**
    * The list of children the group contains.
    */
   private List<YoGraphicDefinition> children;

   /**
    * Creates a new empty group.
    * <p>
    * This constructor is meant for XML parser, to create an empty group prefer using {@link YoGraphicGroupDefinition(String)} or
    * make sure to set the name later when using this constructor.
    * </p>
    */
   public YoGraphicGroupDefinition()
   {
   }

   /**
    * Creates a new empty group and sets its name.
    *
    * @param name the group name.
    */
   public YoGraphicGroupDefinition(String name)
   {
      this();
      setName(name);
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicGroupDefinition(YoGraphicGroupDefinition other)
   {
      super(other);
      if (other.children != null)
         children = other.children.stream().map(YoGraphicDefinition::copy).collect(Collectors.toList());
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerListField("children", this::getChildren, this::setChildren, "child", YoGraphicDefinition::toParsableString, YoGraphicDefinition::parse);
   }

   /**
    * Creates a new group, sets its name, and adds children.
    *
    * @param name     the group name.
    * @param children the initial set of children for the group.
    */
   public YoGraphicGroupDefinition(String name, YoGraphicDefinition... children)
   {
      this(name, Arrays.asList(children));
   }

   /**
    * Creates a new group, sets its name, and adds children.
    *
    * @param name     the group name.
    * @param children the initial set of children for the group.
    */
   public YoGraphicGroupDefinition(String name, Collection<? extends YoGraphicDefinition> children)
   {
      this(name);

      if (children != null)
         this.children = new ArrayList<>(children);
   }

   /**
    * Adds a child to the group.
    * <p>
    * See {@link YoGraphicDefinitionFactory} for factory methods helping with the creation of yoGraphic
    * definitions.
    * </p>
    *
    * @param child the new child.
    */
   public void addChild(YoGraphicDefinition child)
   {
      if (child == null)
         return;

      if (child instanceof YoGraphicListDefinition list)
      {
         if (list == null || list.getYoGraphics() == null)
            return;

         for (int i = 0; i < list.getYoGraphics().size(); i++)
         {
            addChild(list.getYoGraphics().get(i));
         }
         return;
      }

      if (children == null)
         children = new ArrayList<>();
      children.add(child);
   }

   /**
    * Sets the children for the group.
    *
    * @param children the children for the group.
    */
   @XmlElement
   public void setChildren(List<YoGraphicDefinition> children)
   {
      this.children = children;
   }

   /**
    * Unwraps any list registered and proceeds recursively to any registered sub-group(s).
    */
   public void unwrapLists()
   {
      if (children == null)
         return;

      for (int i = children.size() - 1; i >= 0; i--)
      {
         YoGraphicDefinition child = children.get(i);
         if (child instanceof YoGraphicListDefinition list)
         {
            children.remove(i);
            if (list.getYoGraphics() != null)
            {
               list.unwrapNestedLists();
               children.addAll(i, list.getYoGraphics());
            }
         }
         else if (child instanceof YoGraphicGroupDefinition subGroup)
         {
            subGroup.unwrapLists();
         }
      }
   }

   /**
    * Unwraps any sub-group registered. This is useful to flatten the hierarchy starting from this
    * group.
    */
   public void unwrapNestedGroups()
   {
      if (children == null)
         return;

      for (int i = children.size() - 1; i >= 0; i--)
      {
         YoGraphicDefinition child = children.get(i);
         if (child instanceof YoGraphicGroupDefinition subGroup)
         {
            children.remove(i);
            if (subGroup.getChildren() != null)
            {
               subGroup.unwrapNestedGroups();
               children.addAll(i, subGroup.getChildren());
            }
         }
      }
   }

   /**
    * Merges any sub-groups sharing the same name. The operation is repeated recursively through all
    * the descendants.
    */
   public void mergeNestedGroupsByName()
   {
      if (children == null)
         return;

      for (int i = children.size() - 1; i >= 0; i--)
      {
         YoGraphicDefinition child = children.get(i);
         if (child instanceof YoGraphicGroupDefinition subGroup)
         {
            subGroup.mergeNestedGroupsByName();

            for (int j = children.size() - 1; j > i; j--)
            {
               YoGraphicDefinition otherChild = children.get(j);
               if (otherChild instanceof YoGraphicGroupDefinition otherSubGroup)
               {
                  if (subGroup.getName().equals(otherSubGroup.getName()))
                  {
                     children.remove(j);
                     if (otherSubGroup.getChildren() != null)
                        subGroup.getChildren().addAll(otherSubGroup.getChildren());
                  }
               }
            }
         }
         else if (child instanceof YoGraphicListDefinition list)
         {
            list.mergeGroupsByName();
         }
      }
   }

   /**
    * Returns whether the group is empty.
    *
    * @return {@code true} if the list of children either {@code null} or empty.
    */
   public boolean isEmpty()
   {
      return children == null || children.isEmpty();
   }

   public List<YoGraphicDefinition> getChildren()
   {
      return children;
   }

   @Override
   public YoGraphicGroupDefinition copy()
   {
      return new YoGraphicGroupDefinition(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (!super.equals(object))
      {
         return false;
      }
      else if (object instanceof YoGraphicGroupDefinition other)
      {
         return Objects.equals(children, other.children);
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString(int indent)
   {
      String out = "%s [name=%s, visible=%b, children=%s]";
      return out.formatted(getClass().getSimpleName(), name, visible, indentedListString(indent, true, children, child -> child.toString(indent + 1)));
   }
}
