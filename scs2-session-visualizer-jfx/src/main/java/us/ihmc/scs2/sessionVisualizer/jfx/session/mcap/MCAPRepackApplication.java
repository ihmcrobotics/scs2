package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import javafx.application.Platform;
import javafx.stage.FileChooser.ExtensionFilter;
import me.tongfei.progressbar.ProgressBar;
import me.tongfei.progressbar.ProgressBarStyle;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Options;
import org.jetbrains.annotations.NotNull;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.log.ProgressConsumer;
import us.ihmc.scs2.session.mcap.MCAPLogRepacker;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXApplicationCreator;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.time.Duration;
import java.time.temporal.ChronoUnit;

/**
 * Standalone application to repack an MCAP file.
 * <p>
 * Repacking an MCAP file consists in rebuilding the some of the MCAP records only reusing the minimum data from the original file.
 * This can be useful to recover a MCAP file with corrupted timestamps or index references.
 * </p>
 */
public class MCAPRepackApplication
{

   public static void main(String[] args)
   {
      Options options = new Options();
      options.addOption("l", "chunkMin", true, "[Optional] Minimum duration of a chunk in milliseconds.");
      options.addOption("m", "chunkMax", true, "[Optional] Maximum duration of a chunk in milliseconds.");
      options.addOption("i", "input", true, "[Optional] Input file to repack. If not provided a file chooser will be opened.");
      options.addOption("o", "output", true, "[Optional] Output file repacked. If not provided a file chooser will be opened.");
      options.addOption("f", "force", false, "If present, the output file will be overwritten if it already exists.");
      options.addOption("h", "help", false, "Print this message.");

      int minDuration;
      int maxDuration;
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
            String header = "MCAP Repack Application: This application repacks an MCAP file."
                            + " It rebuilds some of the MCAP records only reusing the minimum data from the original file."
                            + " This can be useful to recover a MCAP file with corrupted timestamps or index references.";
            String footer = "Please report issues at https://github.com/ihmcrobotics/simulation-construction-set-2/issues.";
            formatter.printHelp("MCAPRepackApplication", header, options, footer, true);
            System.exit(0);
            return;
         }

         // Access parsed arguments
         minDuration = Integer.parseInt(line.getOptionValue("chunkMin", "50"));
         maxDuration = Integer.parseInt(line.getOptionValue("chunkMax", "500"));
         inputFileName = line.getOptionValue("input");
         outputFileName = line.getOptionValue("output");
         isOverride = line.hasOption("force");

         // Continue with your logic...
      }
      catch (Exception e)
      {
         System.err.println("Parsing failed, use option -h to see usage. Reason: " + e.getMessage());
         System.exit(0);
         return;
      }

      File inputFile = selectInputMCAPFile(inputFileName);
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

      MCAPLogRepacker repacker = new MCAPLogRepacker();
      LogTools.info("Repacking MCAP file: " + inputFile.getAbsolutePath() + " to " + outputFile.getAbsolutePath());

      try (FileInputStream fileInputStream = new FileInputStream(inputFile); FileOutputStream outputStream = new FileOutputStream(outputFile))
      {
         ProgressConsumer progressConsumer = newSimpleProgressConsumer();
         repacker.repack(MCAP.load(fileInputStream.getChannel()), minDuration, maxDuration, outputStream, progressConsumer);
      }
      catch (Exception e)
      {
         LogTools.error("Failed to repack MCAP file: " + e.getMessage());
         e.printStackTrace();
      }
      finally
      {
         System.exit(0);
      }
   }

   @NotNull
   private static ProgressConsumer newSimpleProgressConsumer()
   {
      ProgressBar progressBar = new ProgressBar("Repacking",
                                                100,
                                                10,
                                                false,
                                                true,
                                                System.out,
                                                ProgressBarStyle.COLORFUL_UNICODE_BLOCK,
                                                "",
                                                1L,
                                                false,
                                                null,
                                                ChronoUnit.SECONDS,
                                                0L,
                                                Duration.ZERO);

      return new ProgressConsumer()
      {
         @Override
         public void started(String task)
         {
         }

         @Override
         public void info(String info)
         {
         }

         @Override
         public void error(String error)
         {
         }

         @Override
         public void progress(double progressPercentage)
         {
            progressBar.stepTo(Math.round(progressPercentage * 100.0));
         }

         @Override
         public void done()
         {
            progressBar.stepTo(100);
            ThreadTools.sleep(200);

            LogTools.info("Repacking done.");
         }
      };
   }

   public static File selectInputMCAPFile(String inputFileName)
   {
      return selectInputFile(inputFileName, new ExtensionFilter("MCAP files", "*.mcap"), MCAPLogSessionManagerController.LOG_FILE_KEY);
   }

   public static File selectInputFile(String inputFileName, ExtensionFilter extensionFilter, String key)
   {
      File inputFile = null;

      if (inputFileName != null)
      {
         inputFile = new File(inputFileName);
         if (!inputFile.exists())
         {
            System.err.println("Cannot find input file: " + inputFile.getAbsolutePath());
            return null;
         }
         return inputFile;
      }

      JavaFXApplicationCreator.spawnJavaFXMainApplication();
      Platform.setImplicitExit(false);

      LogTools.info("No input file provided, opening file chooser.");
      inputFile = JavaFXMissingTools.runAndWait(MCAPRepackApplication.class,
                                                () -> SessionVisualizerIOTools.showOpenDialog(null, "Select input file", extensionFilter, key));

      if (inputFile == null)
         System.err.println("No input file selected.");

      return inputFile;
   }

   public static File selectOutputMCAPFile(String outputFileName, boolean isOverride)
   {
      File outputFile = null;

      if (outputFileName != null)
      {
         outputFile = new File(outputFileName);
         if (outputFile.exists())
         {
            if (isOverride)
            {
               outputFile.delete();
            }
            else
            {
               LogTools.error("Output file already exists: " + outputFile.getAbsolutePath() + ". Use the -f option to override it.");
               return null;
            }
         }
         return outputFile;
      }

      JavaFXApplicationCreator.spawnJavaFXMainApplication();
      Platform.setImplicitExit(false);

      LogTools.info("No output file provided, opening file chooser.");
      outputFile = JavaFXMissingTools.runAndWait(MCAPRepackApplication.class,
                                                 () -> SessionVisualizerIOTools.showSaveDialog(null,
                                                                                               "Select output file",
                                                                                               new ExtensionFilter("MCAP files", "*.mcap"),
                                                                                               MCAPLogSessionManagerController.LOG_FILE_KEY));

      if (outputFile == null)
         System.err.println("No output file selected.");
      return outputFile;
   }
}
