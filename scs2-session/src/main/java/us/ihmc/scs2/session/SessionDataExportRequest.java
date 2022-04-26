package us.ihmc.scs2.session;

import java.io.File;
import java.util.function.Predicate;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools.DataFormat;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class contains the information required to request to export session's data to file.
 */
public class SessionDataExportRequest
{
   /** File pointing to the directory in which the session's data will be saved. */
   private File file;
   /** Whether overwriting existing files/folders is allowed. */
   private boolean overwrite = true;
   /** Filter for selecting variables to export. */
   private Predicate<YoVariable> variableFilter = null;
   /** Filter for filtering registries to export. */
   private Predicate<YoRegistry> registryFilter = null;

   /** Whether to export the robot definitions. */
   private boolean exportRobotDefinitions = true;
   /** Whether to export the terrain definitions. */
   private boolean exportTerrainObjectDefinitions = true;
   /** Whether to export the yoGraphic definitions. */
   private boolean exportSessionYoGraphicDefinitions = true;
   /**
    * Whether to export the registry definition.
    * <p>
    * This is required to be able to load the data back into SCS2.
    * </p>
    */
   private boolean exportSessionBufferRegistryDefinition = true;
   /** Whether to export the current robot states. */
   private boolean exportRobotStateDefinitions = true;
   /** The output format for storing the buffered data. */
   private DataFormat exportSessionBufferDataFormat = DataFormat.CSV;

   /** Callback invoked when the export starts. */
   private Runnable onExportStartCallback = null;
   /** Callback invoked when the export ends. */
   private Runnable onExportEndCallback = null;

   /**
    * Creates a new request.
    * <p>
    * The request needs at least to contain a file to be able to export data.
    * </p>
    */
   public SessionDataExportRequest()
   {
   }

   /**
    * [Required] Specifies the folder in which the session data is to be exported.
    * 
    * @param file the file pointing to the directory where to export the session data.
    */
   public void setFile(File file)
   {
      this.file = file;
   }

   /**
    * Whether overwriting any existing file/folder during the export is permitted.
    * 
    * @param overwrite {@code true} to allow overwriting existing file/folder. Default value
    *                  {@code true}.
    */
   public void setOverwrite(boolean overwrite)
   {
      this.overwrite = overwrite;
   }

   /**
    * [Optional] Provides a filter to downselect the {@link YoVariable}s to be exported.
    * <p>
    * A {@link YoVariable} is exported if the given predicate returns {@code true}.
    * </p>
    * 
    * @param variableFilter the {@link YoVariable} filter.
    */
   public void setVariableFilter(Predicate<YoVariable> variableFilter)
   {
      this.variableFilter = variableFilter;
   }

   /**
    * [Optional] Provides a filter to downselect the {@link YoRegistry}s to be exported.
    * <p>
    * A {@link YoRegistry} and its descendants are skipped if the predicate returns {@code false}.
    * </p>
    * 
    * @param registryFilter the {@link YoRegistry} filter.
    */
   public void setRegistryFilter(Predicate<YoRegistry> registryFilter)
   {
      this.registryFilter = registryFilter;
   }

   /**
    * Bundles the flags for exporting the robot, terrain, and yoGraphic definitions.
    * 
    * @param export whether to export the definitions or not.
    */
   public void setExportDefinitions(boolean export)
   {
      setExportRobotDefinitions(export);
      setExportTerrainObjectDefinitions(export);
      setExportSessionYoGraphicDefinitions(export);
   }

   /**
    * Sets whether to export the robot definitions or not.
    * 
    * @param exportRobotDefinitions {@code true} to export the robot definitions, {@code false} to
    *                               skip. Default value {@code true}.
    */
   public void setExportRobotDefinitions(boolean exportRobotDefinitions)
   {
      this.exportRobotDefinitions = exportRobotDefinitions;
   }

   /**
    * Sets whether to export the terrain definitions or not.
    * 
    * @param exportRobotDefinitions {@code true} to export the terrain definitions, {@code false} to
    *                               skip. Default value {@code true}.
    */
   public void setExportTerrainObjectDefinitions(boolean exportTerrainObjectDefinitions)
   {
      this.exportTerrainObjectDefinitions = exportTerrainObjectDefinitions;
   }

   /**
    * Sets whether to export the yoGraphic definitions or not.
    * 
    * @param exportRobotDefinitions {@code true} to export the yoGraphic definitions, {@code false} to
    *                               skip. Default value {@code true}.
    */
   public void setExportSessionYoGraphicDefinitions(boolean exportSessionYoGraphicDefinitions)
   {
      this.exportSessionYoGraphicDefinitions = exportSessionYoGraphicDefinitions;
   }

   /**
    * Sets whether to export the registry definition or not.
    * <p>
    * WARNING: This is required to be able to load the data back into SCS2.
    * </p>
    * 
    * @param exportRobotDefinitions {@code true} to export the registry definition, {@code false} to
    *                               skip. Default value {@code true}.
    */
   public void setExportSessionBufferRegistryDefinition(boolean exportSessionBufferRegistryDefinition)
   {
      this.exportSessionBufferRegistryDefinition = exportSessionBufferRegistryDefinition;
   }

   /**
    * Sets whether to export the current robot states or not.
    * 
    * @param exportRobotDefinitions {@code true} to export the robot states, {@code false} to skip.
    *                               Default value {@code true}.
    */
   public void setExportRobotStateDefinitions(boolean exportRobotStateDefinitions)
   {
      this.exportRobotStateDefinitions = exportRobotStateDefinitions;
   }

   /**
    * Sets the output data format for the buffered data.
    * 
    * @param exportSessionBufferDataFormat the desired format. Default value is {@link DataFormat#CSV}.
    */
   public void setExportSessionBufferDataFormat(DataFormat exportSessionBufferDataFormat)
   {
      this.exportSessionBufferDataFormat = exportSessionBufferDataFormat;
   }

   /**
    * Sets a callback to be notified when the export is starting.
    * 
    * @param onExportStartCallback the callback.
    */
   public void setOnExportStartCallback(Runnable onExportStartCallback)
   {
      this.onExportStartCallback = onExportStartCallback;
   }

   /**
    * Sets a callback to be notified when the export ended.
    * 
    * @param onExportEndCallback the callback.
    */
   public void setOnExportEndCallback(Runnable onExportEndCallback)
   {
      this.onExportEndCallback = onExportEndCallback;
   }

   /**
    * The destination folder where the session data is to be exported.
    * 
    * @return the destination folder.
    */
   public File getFile()
   {
      return file;
   }

   /**
    * Whether overwriting existing file/folder is enabled.
    * 
    * @return {@code true} if overwriting is enabled, {@code false} otherwise.
    */
   public boolean getOverwrite()
   {
      return overwrite;
   }

   /**
    * Gets the filter for selecting variables to export.
    * 
    * @return the variable filter.
    */
   public Predicate<YoVariable> getVariableFilter()
   {
      return variableFilter;
   }

   /**
    * Gets the filter for selecting registries to export.
    * 
    * @return the registry filter.
    */
   public Predicate<YoRegistry> getRegistryFilter()
   {
      return registryFilter;
   }

   /**
    * Whether to export the robot definitions.
    * 
    * @return {@code true} to enable export.
    */
   public boolean getExportRobotDefinitions()
   {
      return exportRobotDefinitions;
   }

   /**
    * Whether to export the terrain definitions.
    * 
    * @return {@code true} to enable export.
    */
   public boolean getExportTerrainObjectDefinitions()
   {
      return exportTerrainObjectDefinitions;
   }

   /**
    * Whether to export the yoGraphic definitions.
    * 
    * @return {@code true} to enable export.
    */
   public boolean getExportSessionYoGraphicDefinitions()
   {
      return exportSessionYoGraphicDefinitions;
   }

   /**
    * Whether to export the registry definition.
    * 
    * @return {@code true} to enable export.
    */
   public boolean getExportSessionBufferRegistryDefinition()
   {
      return exportSessionBufferRegistryDefinition;
   }

   /**
    * Whether to export the current robot states.
    * 
    * @return {@code true} to enable export.
    */
   public boolean getExportRobotStateDefinitions()
   {
      return exportRobotStateDefinitions;
   }

   /**
    * The output format for storing the buffered data.
    * 
    * @return the output format.
    */
   public DataFormat getExportSessionBufferDataFormat()
   {
      return exportSessionBufferDataFormat;
   }

   /**
    * The callback to be notified when the export is starting.
    * 
    * @return the callback.
    */
   public Runnable getOnExportStartCallback()
   {
      return onExportStartCallback;
   }

   /**
    * The callback to be notified when the export ended.
    * 
    * @return the callback.
    */
   public Runnable getOnExportEndCallback()
   {
      return onExportEndCallback;
   }

   @Override
   public String toString()
   {
      return "[file=" + file + ", variableFilter=" + variableFilter + ", registryFilter=" + registryFilter + "]";
   }
}
