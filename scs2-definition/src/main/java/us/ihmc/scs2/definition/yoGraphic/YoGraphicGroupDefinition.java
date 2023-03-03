package us.ihmc.scs2.definition.yoGraphic;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoGraphicGroup")
public class YoGraphicGroupDefinition extends YoGraphicDefinition
{
   private List<YoGraphicDefinition> children;

   public YoGraphicGroupDefinition()
   {
      registerListField("children", this::getChildren, this::setChildren, "child", YoGraphicDefinition::toParsableString, YoGraphicDefinition::parse);
   }

   public YoGraphicGroupDefinition(String name)
   {
      this();
      setName(name);
   }

   public YoGraphicGroupDefinition(String name, YoGraphicDefinition... children)
   {
      this(name, Arrays.asList(children));
   }

   public YoGraphicGroupDefinition(String name, Collection<? extends YoGraphicDefinition> children)
   {
      this(name);

      if (children != null)
         this.children = new ArrayList<>(children);
   }

   public void addChild(YoGraphicDefinition child)
   {
      if (child == null)
         return;
      if (child instanceof YoGraphicListDefinition list)
         addChildren(list);
      if (children == null)
         children = new ArrayList<>();
      children.add(child);
   }

   public void addChildren(YoGraphicListDefinition list)
   {
      if (list == null || list.getYoGraphics() == null)
         return;

      for (int i = 0; i < list.getYoGraphics().size(); i++)
      {
         addChild(list.getYoGraphics().get(i));
      }
   }

   @XmlElement
   public void setChildren(List<YoGraphicDefinition> children)
   {
      this.children = children;
   }

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

   public List<YoGraphicDefinition> getChildren()
   {
      return children;
   }

   public boolean isEmpty()
   {
      return children == null || children.isEmpty();
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
