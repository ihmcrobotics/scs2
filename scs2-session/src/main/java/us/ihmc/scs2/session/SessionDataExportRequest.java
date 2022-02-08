package us.ihmc.scs2.session;

import java.io.File;
import java.util.function.Predicate;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools.DataFormat;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class SessionDataExportRequest
{
   private File file;
   private boolean overwrite = true;
   private int inPoint = -1;
   private int outPoint = -1;
   private int recordPeriod = -1;
   private Predicate<YoVariable> variableFilter = null;
   private Predicate<YoRegistry> registryFilter = null;

   private boolean exportRobotDefinitions = true;
   private boolean exportTerrainObjectDefinitions = true;
   private boolean exportSessionYoGraphicDefinitions = true;
   private boolean exportSessionBufferRegistryDefinition = true;
   private boolean exportRobotStateDefinitions = true;
   private DataFormat exportSessionBufferDataFormat = DataFormat.CSV;

   private Runnable onExportStartCallback = null;
   private Runnable onExportEndCallback = null;

   public SessionDataExportRequest()
   {
   }

   public SessionDataExportRequest(File file,
                                   int inPoint,
                                   int outPoint,
                                   int recordPeriod,
                                   Predicate<YoVariable> variableFilter,
                                   Predicate<YoRegistry> registryFilter)
   {
      this.file = file;
      this.inPoint = inPoint;
      this.outPoint = outPoint;
      this.recordPeriod = recordPeriod;
      this.variableFilter = variableFilter;
      this.registryFilter = registryFilter;
   }

   public void setFile(File file)
   {
      this.file = file;
   }

   public void setOverwrite(boolean overwrite)
   {
      this.overwrite = overwrite;
   }

   public void setInPoint(int inPoint)
   {
      this.inPoint = inPoint;
   }

   public void setOutPoint(int outPoint)
   {
      this.outPoint = outPoint;
   }

   public void setRecordPeriod(int recordPeriod)
   {
      this.recordPeriod = recordPeriod;
   }

   public void setVariableFilter(Predicate<YoVariable> variableFilter)
   {
      this.variableFilter = variableFilter;
   }

   public void setRegistryFilter(Predicate<YoRegistry> registryFilter)
   {
      this.registryFilter = registryFilter;
   }

   public void setExportRobotDefinitions(boolean exportRobotDefinitions)
   {
      this.exportRobotDefinitions = exportRobotDefinitions;
   }

   public void setExportTerrainObjectDefinitions(boolean exportTerrainObjectDefinitions)
   {
      this.exportTerrainObjectDefinitions = exportTerrainObjectDefinitions;
   }

   public void setExportSessionYoGraphicDefinitions(boolean exportSessionYoGraphicDefinitions)
   {
      this.exportSessionYoGraphicDefinitions = exportSessionYoGraphicDefinitions;
   }

   public void setExportSessionBufferRegistryDefinition(boolean exportSessionBufferRegistryDefinition)
   {
      this.exportSessionBufferRegistryDefinition = exportSessionBufferRegistryDefinition;
   }

   public void setExportRobotStateDefinitions(boolean exportRobotStateDefinitions)
   {
      this.exportRobotStateDefinitions = exportRobotStateDefinitions;
   }

   public void setExportSessionBufferDataFormat(DataFormat exportSessionBufferDataFormat)
   {
      this.exportSessionBufferDataFormat = exportSessionBufferDataFormat;
   }

   public void setOnExportStartCallback(Runnable onExportStartCallback)
   {
      this.onExportStartCallback = onExportStartCallback;
   }

   public void setOnExportEndCallback(Runnable onExportEndCallback)
   {
      this.onExportEndCallback = onExportEndCallback;
   }

   public File getFile()
   {
      return file;
   }

   public boolean getOverwrite()
   {
      return overwrite;
   }

   public int getInPoint()
   {
      return inPoint;
   }

   public int getOutPoint()
   {
      return outPoint;
   }

   public int getRecordPeriod()
   {
      return recordPeriod;
   }

   public Predicate<YoVariable> getVariableFilter()
   {
      return variableFilter;
   }

   public Predicate<YoRegistry> getRegistryFilter()
   {
      return registryFilter;
   }

   public boolean getExportRobotDefinitions()
   {
      return exportRobotDefinitions;
   }

   public boolean getExportTerrainObjectDefinitions()
   {
      return exportTerrainObjectDefinitions;
   }

   public boolean getExportSessionYoGraphicDefinitions()
   {
      return exportSessionYoGraphicDefinitions;
   }

   public boolean getExportSessionBufferRegistryDefinition()
   {
      return exportSessionBufferRegistryDefinition;
   }

   public boolean getExportRobotStateDefinitions()
   {
      return exportRobotStateDefinitions;
   }

   public DataFormat getExportSessionBufferDataFormat()
   {
      return exportSessionBufferDataFormat;
   }

   public Runnable getOnExportStartCallback()
   {
      return onExportStartCallback;
   }

   public Runnable getOnExportEndCallback()
   {
      return onExportEndCallback;
   }

   @Override
   public String toString()
   {
      return "[file=" + file + ", inPoint=" + inPoint + ", outPoint=" + outPoint + ", recordPeriod=" + recordPeriod + ", variableFilter=" + variableFilter
            + ", registryFilter=" + registryFilter + "]";
   }
}
