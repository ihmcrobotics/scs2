package us.ihmc.scs2.sharedMemory.interfaces;

import us.ihmc.scs2.sharedMemory.LinkedBufferProperties;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public interface LinkedYoVariableFactory
{
   LinkedBufferProperties newLinkedBufferProperties();

   LinkedYoRegistry newLinkedYoRegistry();

   LinkedYoRegistry newLinkedYoRegistry(YoRegistry registryToLink);

   default LinkedYoVariable<?> newLinkedYoVariable(YoVariable variableToLink)
   {
      return newLinkedYoVariable(variableToLink, null);
   }

   LinkedYoVariable<?> newLinkedYoVariable(YoVariable variableToLink, Object initialUser);
}
