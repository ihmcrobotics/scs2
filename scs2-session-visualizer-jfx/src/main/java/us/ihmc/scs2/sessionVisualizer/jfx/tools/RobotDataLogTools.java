package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.Model;
import us.ihmc.robotDataLogger.Variables;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.LogTimeStampedIndexGenerator;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.ProgressConsumer;

public class RobotDataLogTools
{
   public static File propertyFile(File logDirectory)
   {
      return new File(logDirectory, YoVariableLoggerListener.propertyFile);
   }

   public static File handshakeFile(File logDirectory, LogProperties logProperties, boolean required)
   {
      return handshakeFile(logDirectory, logProperties.getVariables(), required);
   }

   public static File handshakeFile(File logDirectory, Variables variables, boolean required)
   {
      File handshakeFile = new File(logDirectory, variables.getHandshakeAsString());

      if ((required && !handshakeFile.exists()) || handshakeFile.isDirectory())
      {
         throw new RuntimeException("Cannot find " + variables.getHandshakeAsString());
      }

      return handshakeFile;
   }

   public static File logDataFile(File logDirectory, LogProperties logProperties, boolean required)
   {
      return logDataFile(logDirectory, logProperties.getVariables(), required);
   }

   public static File logDataFile(File logDirectory, Variables variables, boolean required)
   {
      File logDataFile = new File(logDirectory, variables.getDataAsString());

      if (required && !logDataFile.exists())
         throw new RuntimeException("Cannot find " + variables.getDataAsString());

      return logDataFile;
   }

   public static File summaryFile(File logDirectory, LogProperties logProperties)
   {
      return summaryFile(logDirectory, logProperties.getVariables());
   }

   public static File summaryFile(File logDirectory, Variables variables)
   {
      if (variables.getSummaryAsString().isEmpty())
         return null;

      return new File(logDirectory, variables.getSummaryAsString());
   }
   
   public static File indexFile(File logDirectory, LogProperties logProperties)
   {
      return indexFile(logDirectory, logProperties.getVariables());
   }
   
   public static File indexFile(File logDirectory, Variables variables)
   {
      if (variables.getIndexAsString().isEmpty())
         return null;
      
      return new File(logDirectory, variables.getIndexAsString());
   }

   public static File modelFile(File logDirectory, LogProperties logProperties)
   {
      return modelFile(logDirectory, logProperties.getModel());
   }

   public static File modelFile(File logDirectory, Model model)
   {
      if (model.getPathAsString().isEmpty())
         return null;

      return new File(logDirectory, model.getPathAsString());
   }

   public static File modelResourceBundleFile(File logDirectory, LogProperties logProperties)
   {
      return modelResourceBundleFile(logDirectory, logProperties.getModel());
   }

   public static File modelResourceBundleFile(File logDirectory, Model model)
   {
      if (model.getResourceBundleAsString().isEmpty())
         return null;

      return new File(logDirectory, model.getResourceBundleAsString());
   }

   public static YoVariableHandshakeParser parseYoVariables(File logDirectory, LogProperties logProperties) throws IOException
   {
      return parseYoVariables(logDirectory, logProperties.getVariables());
   }

   public static YoVariableHandshakeParser parseYoVariables(File logDirectory, Variables variables) throws IOException
   {
      File handshakeFile = handshakeFile(logDirectory, variables, true);
      YoVariableHandshakeParser parser = YoVariableHandshakeParser.create(variables.getHandshakeFileType());
      parser.parseFrom(readResourceFile(handshakeFile));
      return parser;
   }

   public static byte[] loadModel(File logDirectory, LogProperties logProperties) throws IOException
   {
      return loadModel(logDirectory, logProperties.getModel());
   }

   public static byte[] loadModel(File logDirectory, Model model) throws IOException
   {
      return readResourceFile(modelFile(logDirectory, model));
   }

   public static byte[] loadModelResourceBundle(File logDirectory, LogProperties logProperties) throws IOException
   {
      return loadModelResourceBundle(logDirectory, logProperties.getModel());
   }

   public static byte[] loadModelResourceBundle(File logDirectory, Model model) throws IOException
   {
      return readResourceFile(modelResourceBundleFile(logDirectory, model));
   }

   public static byte[] readResourceFile(File file) throws IOException
   {
      if (file == null)
         return null;
      DataInputStream modelStream = new DataInputStream(new FileInputStream(file));
      byte[] modelData = new byte[(int) file.length()];
      modelStream.readFully(modelData);
      modelStream.close();
      return modelData;
   }

   public static RobotDefinition loadRobotDefinition(File logDirectory, LogProperties logProperties) throws IOException
   {
      return loadRobotDefinition(logDirectory, logProperties.getModel());
   }

   public static RobotDefinition loadRobotDefinition(File logDirectory, Model model) throws IOException
   {
      byte[] modelData = loadModel(logDirectory, model);
      if (modelData == null)
         return null;
      byte[] resourceData = loadModelResourceBundle(logDirectory, model);
      if (resourceData == null)
         return null;
      return RobotModelLoader.loadModel(model.getNameAsString(), model.getResourceDirectoriesList().toStringArray(), modelData, resourceData);
   }

   public static void updateLogs(File directory, LogProperties properties, ProgressConsumer progressConsumer)
   {
      Variables variables = properties.getVariables();

      try
      {
         if (variables.getCompressed() && !variables.getTimestamped())
         {
            LogTimeStampedIndexGenerator.convert(directory, properties, progressConsumer);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException("Cannot convert log file", e);
      }
   }
}
