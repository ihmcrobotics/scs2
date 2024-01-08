package us.ihmc.scs2.sessionVisualizer.jfx;

import com.martiansoftware.jsap.*;
import org.apache.commons.io.FilenameUtils;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.scs2.session.mcap.MCAPLogSession;

import java.io.File;

/**
 * This class is used to parse the command line arguments for the {@link SessionVisualizer}.
 */
public class SessionVisualizerArgsHandler
{
   private Session session = null;

   /**
    * Parses the command line arguments.
    *
    * @param args the command line arguments.
    * @return {@code true} if the arguments were parsed successfully, {@code false} otherwise.
    * @throws Exception if an exception occurs while parsing the arguments.
    */
   public boolean parseArgs(String[] args) throws Exception
   {
      String logFileOption = "logFileName";
      String desiredDTOption = "desiredDT";
      String defaultRobotFileOption = "defaultRobotFileName";
      SimpleJSAP jsap = new SimpleJSAP("SCS2 Session Visualizer",
                                       "Visualizes a robot log file, or live data from a compatible source.",
                                       new Parameter[] {new FlaggedOption(logFileOption,
                                                                          JSAP.STRING_PARSER,
                                                                          null,
                                                                          JSAP.NOT_REQUIRED,
                                                                          'l',
                                                                          "log",
                                                                          "Log file to load, can either be a SCS2 log file or a MCAP log file."),
                                                        new FlaggedOption(desiredDTOption,
                                                                          JSAP.DOUBLE_PARSER,
                                                                          "0.001",
                                                                          JSAP.NOT_REQUIRED,
                                                                          't',
                                                                          "dt",
                                                                          "If possible, the desired DT in seconds to use for the session visualizer. Default value is 1 millisecond."),
                                                        new FlaggedOption(defaultRobotFileOption,
                                                                          JSAP.STRING_PARSER,
                                                                          null,
                                                                          JSAP.NOT_REQUIRED,
                                                                          'r',
                                                                          "robot",
                                                                          "Default robot file to load in case the log file does not contain any robot definition. Can be either a URDF or SDF file.")});
      JSAPResult config = jsap.parse(args);

      if (jsap.messagePrinted())
      {
         System.out.println(jsap.getUsage());
         System.out.println(jsap.getHelp());
         return false;
      }

      String logFileName = config.getString(logFileOption);
      long desiredDT = (long) (1.0e9 * config.getDouble(desiredDTOption));
      String defaultRobotFileName = config.getString(defaultRobotFileOption);

      if (logFileName != null)
      {
         File logFile = new File(logFileName);

         if (!logFile.exists())
         {
            System.err.println("Cannot find log file: " + logFile.getAbsolutePath());
            return false;
         }

         if (FilenameUtils.getExtension(logFileName).equals("mcap"))
         {
            File defaultRobotFile;
            if (defaultRobotFileName == null)
            {
               defaultRobotFile = null;
            }
            else
            {
               defaultRobotFile = new File(defaultRobotFileName);

               if (!defaultRobotFile.exists())
               {
                  System.err.println("Cannot find default robot file: " + defaultRobotFile.getAbsolutePath());
                  return false;
               }
            }

            session = new MCAPLogSession(logFile, desiredDT, defaultRobotFile);
         }
         else if (logFileName.equals(YoVariableLoggerListener.propertyFile))
         {
            session = new LogSession(logFile.getParentFile(), null);
         }
         else if (logFile.isDirectory())
         {
            File[] result = logFile.listFiles((dir, name) -> name.equals(YoVariableLoggerListener.propertyFile));
            if (result == null || result.length == 0)
            {
               System.err.println("Cannot find log file: " + logFile.getAbsolutePath());
               return false;
            }
            else
            {
               session = new LogSession(logFile, null);
            }
         }
         else
         {
            System.err.println("Unknown log file type: " + logFile.getAbsolutePath());
            return false;
         }
      }

      return true;
   }

   public Session getSession()
   {
      return session;
   }
}
