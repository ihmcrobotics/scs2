package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import javafx.stage.FileChooser.ExtensionFilter;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import us.ihmc.log.LogTools;

import java.io.File;

import static us.ihmc.scs2.sessionVisualizer.jfx.session.mcap.MCAPRepackApplication.selectOutputMCAPFile;

public class IHMCLogConverter
{
   public static void main(String[] args)
   {
      Options options = new Options();
      options.addOption("i", "input", true, "[Optional] Input file to convert. If not provided a file chooser will be opened.");
      options.addOption("o", "output", true, "[Optional] Output file converted. If not provided a file chooser will be opened.");
      options.addOption("f", "force", false, "If present, the output file will be overwritten if it already exists.");
      options.addOption("h", "help", false, "Print this message.");

      String inputFileName;
      String outputFileName;
      boolean isOverride;

      // Create a parser
      CommandLineParser parser = new DefaultParser();
      try
      {
         // Parse the command line arguments
         CommandLine line = parser.parse(options, args);

         if (line.hasOption("help"))
         {
            HelpFormatter formatter = new HelpFormatter();
            String header = "IHMC Log Converter: This application converts an IHMC log into a MCAP log file.";
            String footer = "Please report issues at https://github.com/ihmcrobotics/simulation-construction-set-2/issues.";
            formatter.printHelp("IHMCLogConverter", header, options, footer, true);
            System.exit(0);
            return;
         }

         // Access parsed arguments
         inputFileName = line.getOptionValue("input");
         outputFileName = line.getOptionValue("output");
         isOverride = line.hasOption("force");
      }
      catch (ParseException e)
      {
         System.err.println("Parsing failed, use option -h to see usage. Reason: " + e.getMessage());
         System.exit(0);
         return;
      }

      File inputFile = selectInputIHMCLogFile(inputFileName);
      if (inputFile == null)
      {
         System.exit(0);
         return;
      }

      File outputFile = selectOutputMCAPFile(outputFileName, isOverride);
      if (outputFile == null)
      {
         System.exit(0);
         return;
      }

      LogTools.info("Converting IHMC log file: " + inputFile.getAbsolutePath() + " to MCAP log file: " + outputFile.getAbsolutePath());
      IHMCLogToMCAPConverter.convertIHMCLogToMCAP(inputFile, outputFile);
   }

   public static File selectInputIHMCLogFile(String inputFileName)
   {
      return MCAPRepackApplication.selectInputFile(inputFileName,
                                                   new ExtensionFilter("Log property file", "*.log"),
                                                   MCAPLogSessionManagerController.LOG_FILE_KEY);
   }
}
