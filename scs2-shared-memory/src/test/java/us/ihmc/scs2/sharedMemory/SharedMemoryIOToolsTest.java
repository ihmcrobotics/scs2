package us.ihmc.scs2.sharedMemory;

import static us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools.MATLAB_VARNAME_MAX_LENGTH;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Random;

import javax.xml.bind.JAXBException;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTestTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SharedMemoryIOToolsTest
{
   @Test
   public void testExportImportRegistry() throws JAXBException, IOException
   {
      Random random = new Random(34536);

      String registryFileName = "./bufferASCIIExport.scs2.registry";
      YoRegistry exportedRegistry = SharedMemoryRandomTools.nextYoRegistryTree(random, 20, 20)[0];
      SharedMemoryIOTools.exportRegistry(exportedRegistry, new FileOutputStream(registryFileName));

      YoRegistry importedRegistry = SharedMemoryIOTools.importRegistry(new FileInputStream(registryFileName));

      SharedMemoryTestTools.assertYoRegistryEquals(exportedRegistry, importedRegistry);

      Files.delete(Paths.get(registryFileName));
   }

   @Test
   public void testExportImportASCII() throws JAXBException, IOException
   {
      Random random = new Random(35453);

      String dataFileName = "./bufferASCIIExport.scs2.data";
      String registryFileName = "./bufferASCIIExport.scs2.registry";

      for (int i = 0; i < 100; i++)
      {
         YoSharedBuffer exportedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 20, 20);
         SharedMemoryIOTools.exportRegistry(exportedBuffer.getRootRegistry(), new FileOutputStream(registryFileName));
         SharedMemoryIOTools.exportDataASCII(exportedBuffer, new FileOutputStream(dataFileName));

         YoRegistry importedRoot = SharedMemoryIOTools.importRegistry(new FileInputStream(registryFileName));
         SharedMemoryTestTools.assertYoRegistryEquals(exportedBuffer.getRootRegistry(), importedRoot);
         YoSharedBuffer importedBuffer = SharedMemoryIOTools.importDataASCII(new FileInputStream(dataFileName), importedRoot);

         SharedMemoryTestTools.assertYoSharedBufferEquals(exportedBuffer, importedBuffer, 0.0);
      }

      Files.delete(Paths.get(dataFileName));
      Files.delete(Paths.get(registryFileName));
   }

   @Test
   public void testExportImportCSV() throws JAXBException, IOException
   {
      Random random = new Random(35453);

      String dataFileName = "./bufferCSVExport.scs2.data";
      String registryFileName = "./bufferCSVExport.scs2.registry";

      for (int i = 0; i < 10; i++)
      {
         YoSharedBuffer exportedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 20, 20);
         SharedMemoryIOTools.exportRegistry(exportedBuffer.getRootRegistry(), new FileOutputStream(registryFileName));
         SharedMemoryIOTools.exportDataCSV(exportedBuffer, new FileOutputStream(dataFileName));

         YoRegistry importedRoot = SharedMemoryIOTools.importRegistry(new FileInputStream(registryFileName));
         SharedMemoryTestTools.assertYoRegistryEquals(exportedBuffer.getRootRegistry(), importedRoot);
         YoSharedBuffer importedBuffer = SharedMemoryIOTools.importDataCSV(new FileInputStream(dataFileName), importedRoot);

         SharedMemoryTestTools.assertYoSharedBufferEquals(exportedBuffer, importedBuffer, 0.0);
      }

      Files.delete(Paths.get(dataFileName));
      Files.delete(Paths.get(registryFileName));
   }

   @Test
   public void testExportImportMatlab() throws JAXBException, IOException
   {
      Random random = new Random(35453);

      String dataFileName = "./bufferMatlabExport.scs2.mat";
      String registryFileName = "./bufferMatlabExport.scs2.registry";

      for (int i = 0; i < 100; i++)
      {
         YoSharedBuffer exportedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, 20, 20);
         SharedMemoryIOTools.exportRegistry(exportedBuffer.getRootRegistry(), new FileOutputStream(registryFileName));
         SharedMemoryIOTools.exportDataMatlab(exportedBuffer, new File(dataFileName));

         YoRegistry importedRoot = SharedMemoryIOTools.importRegistry(new FileInputStream(registryFileName));
         SharedMemoryTestTools.assertYoRegistryEquals(exportedBuffer.getRootRegistry(), importedRoot);
         YoSharedBuffer importedBuffer = SharedMemoryIOTools.importDataMatlab(new File(dataFileName), importedRoot);

         exportedBuffer.cropBuffer(new CropBufferRequest(exportedBuffer.getProperties().getInPoint(), exportedBuffer.getProperties().getOutPoint()));

         SharedMemoryTestTools.assertYoSharedBufferEquals(exportedBuffer, importedBuffer, 0.0);
      }

      Files.delete(Paths.get(dataFileName));
      Files.delete(Paths.get(registryFileName));
   }

   @Test
   public void testExportImportMatlabLongName() throws JAXBException, IOException
   {
      Random random = new Random(35453);

      String dataFileName = "./bufferMatlabExport.scs2.mat";
      String registryFileName = "./bufferMatlabExport.scs2.registry";

      for (int i = 0; i < 100; i++)
      {
         YoRegistry exportedRoot = SharedMemoryRandomTools.nextYoRegistryTree(random, 20, 20)[0];
         new YoDouble(SharedMemoryRandomTools.nextAlphanumericString(random, MATLAB_VARNAME_MAX_LENGTH + 1, MATLAB_VARNAME_MAX_LENGTH + 100), exportedRoot);
         YoRegistry longNameRegistry = new YoRegistry(SharedMemoryRandomTools.nextAlphanumericString(random,
                                                                                                     MATLAB_VARNAME_MAX_LENGTH + 1,
                                                                                                     MATLAB_VARNAME_MAX_LENGTH + 100));
         exportedRoot.addChild(longNameRegistry);
         new YoDouble("bloppy", longNameRegistry);

         YoSharedBuffer exportedBuffer = SharedMemoryRandomTools.nextYoSharedBuffer(random, exportedRoot);
         SharedMemoryIOTools.exportRegistry(exportedRoot, new FileOutputStream(registryFileName));
         SharedMemoryIOTools.exportDataMatlab(exportedBuffer, new File(dataFileName));

         YoRegistry importedRoot = SharedMemoryIOTools.importRegistry(new FileInputStream(registryFileName));
         SharedMemoryTestTools.assertYoRegistryEquals(exportedRoot, importedRoot);
         YoSharedBuffer importedBuffer = SharedMemoryIOTools.importDataMatlab(new File(dataFileName), importedRoot);

         exportedBuffer.cropBuffer(new CropBufferRequest(exportedBuffer.getProperties().getInPoint(), exportedBuffer.getProperties().getOutPoint()));

         SharedMemoryTestTools.assertYoSharedBufferEquals(exportedBuffer, importedBuffer, 0.0);
      }

      Files.delete(Paths.get(dataFileName));
      Files.delete(Paths.get(registryFileName));
   }
}
