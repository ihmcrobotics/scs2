package us.ihmc.scs2.filtering;

import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.List;
import java.util.Map;

public interface YoFilter
{
   YoVariable getFilteredVariable();

   List<YoVariable> getYoComponents();

   String getName();

   YoNamespace getNamespace();

   default String getFullname()
   {
      return getNamespace().toString() + "." + getName();
   }
}
