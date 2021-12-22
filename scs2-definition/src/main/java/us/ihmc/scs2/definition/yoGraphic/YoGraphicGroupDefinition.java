package us.ihmc.scs2.definition.yoGraphic;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

@XmlRootElement(name = "YoGraphicGroup")
public class YoGraphicGroupDefinition extends YoGraphicDefinition
{
   private List<YoGraphicDefinition> children;

   public YoGraphicGroupDefinition()
   {
   }

   public YoGraphicGroupDefinition(String name)
   {
      super(name);
   }

   public YoGraphicGroupDefinition(String name, YoGraphicDefinition... children)
   {
      this(name, Arrays.asList(children));
   }

   public YoGraphicGroupDefinition(String name, Collection<? extends YoGraphicDefinition> children)
   {
      super(name);

      if (children != null)
         this.children = new ArrayList<>(children);
   }

   @XmlElement
   public void setChildren(List<YoGraphicDefinition> children)
   {
      this.children = children;
   }

   public List<YoGraphicDefinition> getChildren()
   {
      return children;
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
      else if (object instanceof YoGraphicGroupDefinition)
      {
         return children.equals(((YoGraphicGroupDefinition) object).getChildren());
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getCollectionString("\n", children, YoGraphicDefinition::toString);
   }
}
