package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class YoCompositeCollection
{
   private final YoCompositePattern pattern;
   private final List<YoComposite> yoComposites;
   private final Map<String, YoComposite> uniqueNameToYoComposite;
   private final Map<String, YoComposite> uniqueShortNameToYoComposite;
   private final Map<String, YoComposite> fullnameToYoComposite;

   public YoCompositeCollection(YoCompositePattern pattern, List<YoComposite> yoComposites)
   {
      this.pattern = pattern;
      this.yoComposites = yoComposites;
      YoComposite.computeUniqueNames(yoComposites);
      uniqueNameToYoComposite = yoComposites.stream().collect(Collectors.toMap(YoComposite::getUniqueName, Function.identity()));
      uniqueShortNameToYoComposite = yoComposites.stream().collect(Collectors.toMap(YoComposite::getUniqueShortName, Function.identity()));
      fullnameToYoComposite = yoComposites.stream().collect(Collectors.toMap(YoComposite::getFullname, Function.identity()));
   }

   public YoCompositePattern getPattern()
   {
      return pattern;
   }

   public List<YoComposite> getYoComposites()
   {
      return yoComposites;
   }

   public List<YoComposite> getYoComposite(YoRegistry owner)
   {
      return yoComposites.stream().filter(composite -> composite.getNamespace().equals(owner.getNamespace())).collect(Collectors.toList());
   }

   public String getYoVariableUniqueName(YoVariable yoVariable)
   {
      YoComposite yoTypeReference = fullnameToYoComposite.get(yoVariable.getFullNameString());
      return yoTypeReference != null ? yoTypeReference.getUniqueName() : null;
   }

   public String getYoVariableUniqueShortName(YoVariable yoVariable)
   {
      YoComposite yoTypeReference = fullnameToYoComposite.get(yoVariable.getFullNameString());
      return yoTypeReference != null ? yoTypeReference.getUniqueShortName() : null;
   }

   public YoComposite getYoCompositeFromUniqueName(String uniqueName)
   {
      YoComposite result = uniqueNameToYoComposite.get(uniqueName);
      if (result == null)
         result = uniqueShortNameToYoComposite.get(uniqueName);
      return result;
   }

   public YoComposite getYoCompositeFromFullname(String fullname)
   {
      return fullnameToYoComposite.get(fullname);
   }

   public Collection<String> uniqueNameCollection()
   {
      return uniqueNameToYoComposite.keySet();
   }

   public Collection<String> fullnameCollection()
   {
      return fullnameToYoComposite.keySet();
   }
}