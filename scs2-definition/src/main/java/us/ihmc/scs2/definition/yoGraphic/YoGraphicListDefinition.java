package us.ihmc.scs2.definition.yoGraphic;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoGraphicList")
public class YoGraphicListDefinition
{
   private List<YoGraphicDefinition> yoGraphics = new ArrayList<>();

   public YoGraphicListDefinition()
   {
   }

   public YoGraphicListDefinition(Collection<? extends YoGraphicDefinition> yoGraphics)
   {
      setYoGraphics(new ArrayList<>(yoGraphics));
   }

   @XmlElement(name = "yoGraphic")
   public void setYoGraphics(List<YoGraphicDefinition> yoGraphics)
   {
      this.yoGraphics = yoGraphics;
   }

   public List<YoGraphicDefinition> getYoGraphics()
   {
      return yoGraphics;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   public String toString(int indent)
   {
      return YoGraphicDefinition.indentedListString(indent, false, yoGraphics, yoGraphic -> yoGraphic.toString(indent + 1));
   }
}
