package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;

/**
 * Writes a tiny uncompressed Yo-variable log directory that {@code LogSession} can open.
 */
final class SessionControlsTestLog
{
   static final String SESSION_NAME = "cli-bind-log";
   static final String TIMESTAMP = "20260101_120000";

   private SessionControlsTestLog()
   {
   }

   static File createLogDirectory(String directoryName) throws IOException
   {
      return createLogDirectory(directoryName, SESSION_NAME);
   }

   static File createLogDirectory(String directoryName, String sessionName) throws IOException
   {
      File logDirectory = Files.createTempDirectory(directoryName).toFile();
      logDirectory.deleteOnExit();

      Files.writeString(new File(logDirectory, "robotData.log").toPath(), """
            version=4.0
            name=%s
            variables.handshakeFileType=IDL_YAML
            variables.handshake=handshake.yaml
            variables.data=robotData.data
            variables.summary=
            variables.index=
            variables.timestamped=true
            variables.compressed=false
            timestamp=%s
            video.hasTimebase=true
            """.formatted(sessionName, TIMESTAMP));

      Files.writeString(new File(logDirectory, "handshake.yaml").toPath(), """
            us::ihmc::robotDataLogger::Handshake:
              dt: 0.001
              registries:
              - parent: 0
                name: root
              - parent: 0
                name: testReg
              variables:
              - name: value
                description: ''
                type: DoubleYoVariable
                registry: 1
                enumType: 0
                allowNullValues: false
                isParameter: false
                min: 0.0
                max: 1.0
                loadStatus: NoParameter
              joints: []
              graphicObjects: []
              artifacts: []
              scs2YoGraphicDefinitions: []
              enumTypes: []
              referenceFrameInformation:
                frameIndices:
                - 0
                frameNames:
                - World
              summary:
                createSummary: false
                summaryTriggerVariable: ''
                summarizedVariables: []
            """);

      // Uncompressed tick is timestamp + one YoVariable, 8 bytes each. LogDataReader reports
      // (fileSize / tickSize) - 1 entries, so two ticks yield one readable entry.
      int tickSize = 16;
      ByteBuffer data = ByteBuffer.allocate(tickSize * 2).order(ByteOrder.BIG_ENDIAN);
      data.putLong(1_000_000L);
      data.putLong(Double.doubleToLongBits(1.0));
      data.putLong(2_000_000L);
      data.putLong(Double.doubleToLongBits(2.0));
      Files.write(new File(logDirectory, "robotData.data").toPath(), data.array());

      return logDirectory;
   }
}
