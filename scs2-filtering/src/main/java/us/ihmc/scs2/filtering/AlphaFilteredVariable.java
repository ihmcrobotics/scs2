package us.ihmc.scs2.filtering;

import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sharedMemory.YoRegistryBuffer;
import us.ihmc.scs2.sharedMemory.YoVariableBuffer;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.List;

public class AlphaFilteredVariable implements YoFilter
{
   private final YoDouble variableToFilter;
   private final YoVariableBuffer<YoDouble> variableBufferToFilter;
   private final YoVariableBuffer<YoDouble> filteredVariableBuffer;
   private final YoVariableBuffer<YoDouble> alphaBuffer;
   private final YoDouble alphaAmount;
   private final YoDouble filteredVariable;

   private final YoRegistryBuffer registryBuffer;

   public AlphaFilteredVariable(String variableName, YoRegistryBuffer registryBuffer)
   {
      this.registryBuffer = registryBuffer;

      YoRegistry userRegistry = registryBuffer.getRootRegistry();
      variableToFilter = (YoDouble) userRegistry.findVariable(variableName);
      alphaAmount = new YoDouble(variableName + "_Alpha", userRegistry);
      filteredVariable = new YoDouble(variableName + "_AlphaFiltered", userRegistry);
      variableBufferToFilter = (YoVariableBuffer<YoDouble>) registryBuffer.findYoVariableBuffer(variableToFilter);
      filteredVariableBuffer = (YoVariableBuffer<YoDouble>) registryBuffer.findOrCreateYoVariableBuffer(filteredVariable);
      alphaBuffer = (YoVariableBuffer<YoDouble>) registryBuffer.findOrCreateYoVariableBuffer(alphaAmount);

      alphaAmount.set(1.0);
      alphaAmount.addListener(v -> updateAllValues());
      // TODO I don't want to attach a listener here. This will cause issues when scrubbing.
      variableToFilter.addListener(v -> update());
      updateAllValues();
   }

   private void updateAllValues()
   {
      int bufferStart = registryBuffer.getProperties().getInPoint();
      int length = registryBuffer.getProperties().getActiveBufferLength();
      int size = registryBuffer.getProperties().getSize();

      double alpha = MathTools.clamp(alphaAmount.getValue(), 0.0, 1.0);
      filteredVariableBuffer.getAsDoubleBuffer()[bufferStart] = variableBufferToFilter.getAsDoubleBuffer()[bufferStart];
      alphaBuffer.getAsDoubleBuffer()[bufferStart] = alpha;
      int bufferIndex = bufferStart + 1;
      int previousIndex = bufferStart;
      for (int i = 0; i < length; i++)
      {
         if (bufferIndex == size)
         {
            bufferIndex = 0;
         }
         filteredVariableBuffer.getAsDoubleBuffer()[bufferIndex] = (1.0 - alpha) * filteredVariableBuffer.getAsDoubleBuffer()[previousIndex] +
                                                                   alpha * variableBufferToFilter.getAsDoubleBuffer()[bufferIndex];
         alphaBuffer.getAsDoubleBuffer()[bufferIndex] = alpha;
         previousIndex = bufferIndex;
         bufferIndex++;
      }
      filteredVariable.set(filteredVariableBuffer.getAsDoubleBuffer()[registryBuffer.getProperties().getCurrentIndex()]);
   }

   public void update()
   {
      double previousValue = filteredVariable.getValue();
      double alpha = MathTools.clamp(alphaAmount.getValue(), 0.0, 1.0);

      filteredVariable.set((1.0 - alpha) * previousValue + alpha * variableToFilter.getValue());
      filteredVariableBuffer.writeBuffer();
   }

   @Override
   public YoDouble getFilteredVariable()
   {
      return filteredVariable;
   }

   @Override
   public String getName()
   {
      return filteredVariable.getName();
   }

   @Override
   public YoNamespace getNamespace()
   {
      return filteredVariable.getNamespace();
   }

   @Override
   public List<YoVariable> getYoComponents()
   {
      return List.of(alphaAmount, filteredVariable);
   }
}
