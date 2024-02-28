package us.ihmc.scs2.sessionVisualizer.jfx;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Options;
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
      Options options = new Options();
      options.addOption("l", "log", true, "Log file to load, can either be a SCS2 log file or a MCAP log file.");
      options.addOption("t", "dt", true, "If possible, the desired DT in seconds to use for the session visualizer. Default value is 1 millisecond.");
      options.addOption("r",
                        "robot",
                        true,
                        "Default robot file to load in case the log file does not contain any robot definition. Can be either a URDF or SDF file.");
      options.addOption("h", "help", false, "Print this message.");

      CommandLineParser parser = new DefaultParser();
      String logFileName;
      long desiredDT;
      String defaultRobotFileName;
      try
      {
         CommandLine line = parser.parse(options, args);

         if (line.hasOption("help"))
         {
            String header = "SCS2 SessionVisualizer Application: This application is used to visualize log files.";
            String footer = "Please report issues at https://github.com/ihmcrobotics/simulation-construction-set-2/issues.";
            new HelpFormatter().printHelp("SCS2 - SessionVisualizer", header, options, footer, true);
            return false;
         }

         logFileName = line.getOptionValue("log");
         desiredDT = (long) (1.0e9 * Double.parseDouble(line.getOptionValue("dt", "0.001")));
         defaultRobotFileName = line.getOptionValue("robot");
      }
      catch (Exception e)
      {
         System.err.println("Parsing failed, use option -h to see usage. Reason: " + e.getMessage());
         return false;
      }

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
