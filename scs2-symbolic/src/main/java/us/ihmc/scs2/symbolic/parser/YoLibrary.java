package us.ihmc.scs2.symbolic.parser;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.List;

public class YoLibrary
{
   private final List<YoRegistry> yoRegistries = new ArrayList<>();

   public YoLibrary()
   {
   }

   public void addRegistry(YoRegistry yoRegistry)
   {
      yoRegistries.add(yoRegistry);
   }

   public YoLibrary duplicate()
   {
      YoLibrary duplicate = new YoLibrary();
      duplicate.yoRegistries.addAll(yoRegistries);
      return duplicate;
   }

   public YoDouble searchYoDouble(String name)
   {
      return searchYoVariable(YoDouble.class, name);
   }

   public YoInteger searchYoInteger(String name)
   {
      return searchYoVariable(YoInteger.class, name);
   }

   public YoVariable searchYoVariable(String name)
   {
      return searchYoVariable(YoVariable.class, name);
   }

   @SuppressWarnings("unchecked")
   public <T extends YoVariable> T searchYoVariable(Class<T> yoType, String name)
   {
      int separatorIndex = name.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      String namespaceEnding = separatorIndex == -1 ? null : name.substring(0, separatorIndex);
      String variableName = separatorIndex == -1 ? name : name.substring(separatorIndex + 1);

      for (YoRegistry yoRegistry : yoRegistries)
      {
         T yoVariable = (T) YoSearchTools.findFirstVariable(namespaceEnding, variableName, yoType::isInstance, yoRegistry);

         if (yoVariable != null)
            return yoVariable;
      }
      return null;
   }
}
