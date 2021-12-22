package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoComposite implements Comparable<YoComposite>
{
   private final YoCompositePattern pattern;
   private final String name;
   private final YoNamespace namespace;
   private final List<YoVariable> yoComponents;

   private String uniqueName;

   public YoComposite(YoCompositePattern pattern, YoVariable yoVariable)
   {
      this(pattern, yoVariable.getName(), yoVariable.getNamespace(), Collections.singletonList(yoVariable));
   }

   public YoComposite(YoCompositePattern pattern, String compositeName, YoNamespace compositeNamespace, List<YoVariable> yoComponents)
   {
      this.pattern = pattern;
      this.name = compositeName;
      this.namespace = compositeNamespace;
      this.yoComponents = yoComponents;
   }

   public YoCompositePattern getPattern()
   {
      return pattern;
   }

   public String getName()
   {
      return name;
   }

   public YoNamespace getNamespace()
   {
      return namespace;
   }

   public String getUniqueName()
   {
      return uniqueName;
   }

   public String getFullname()
   {
      return namespace.toString() + "." + name;
   }

   public List<YoVariable> getYoComponents()
   {
      return yoComponents;
   }

   @Override
   public int compareTo(YoComposite oher)
   {
      return name.compareTo(oher.name);
   }

   @Override
   public String toString()
   {
      return uniqueName != null ? uniqueName : name;
   }

   public static void computeUniqueNames(List<YoComposite> yoComposites)
   {
      Map<YoComposite, String> yoCompositeToUniqueNameMap = YoCompositeTools.computeUniqueNames(yoComposites,
                                                                                                yoComposite -> yoComposite.getNamespace().getSubNames(),
                                                                                                YoComposite::getName);
      yoCompositeToUniqueNameMap.entrySet().forEach(e -> e.getKey().uniqueName = e.getValue());
   }
}
