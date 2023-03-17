package us.ihmc.scs2.ros;

import java.util.HashSet;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoVariableTools
{
   public static String[] hackNamespaceDuplicates(String[] namespaces)
   {
      HashSet<String> set = new HashSet<>();

      String[] updated = new String[namespaces.length];

      for (int i = 0; i < namespaces.length; i++)
      {
         String namespace = namespaces[i];

         while (set.contains(namespace))
         {
            namespace = "_" + namespace;
         }
         set.add(namespace);
         updated[i] = namespace;
      }
      return updated;
   }
   
   public static YoRegistry getOrCreateRegistry(YoRegistry registry, String[] namespaces)
   {
      namespaces = hackNamespaceDuplicates(namespaces);

      YoRegistry currentRegistry = registry;

      for (int i = 0; i < namespaces.length; i++)
      {
         String namespace = namespaces[i];

         if (namespace != "")
         {
            if (registry.getNamespace().getSubNames().contains(namespace))
            {
               namespace = "_" + namespace;
            }

            YoRegistry child = currentRegistry.getChild(namespace);
            if (child == null)
            {
               child = new YoRegistry(namespace);
               currentRegistry.addChild(child, false);
            }
            currentRegistry = child;
         }
      }
      return currentRegistry;
   }
   
   public static YoDouble getOrCreateYoDouble(YoRegistry registry, String name)
   {
      YoDouble variable = (YoDouble) registry.getVariable(name);
      
      if(variable == null)
      {
         variable =  new YoDouble(name, registry);
      }
      
      return variable;
   }
}
