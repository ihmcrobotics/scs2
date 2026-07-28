package us.ihmc.scs2.sharedMemory;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Not a correctness test - a repeatable microbenchmark for {@link YoVariableBufferList#writeBufferAt(int)}, meant to
 * be run once as-is, then again after {@code git stash}-ing back to the pre-optimization version (or any other
 * candidate change) on the same machine, to compare wall-clock cost.
 * <p>
 * Results print to stdout (run with {@code -i}/{@code --info}, or from an IDE, to see them inline) and are also
 * appended to {@code build/benchmark/writeBufferAt.txt} so they survive whatever the test runner does with stdout -
 * that file is what you diff between runs.
 */
public class YoVariableBufferListWriteBenchmarkTest
{
   private static final int NUMBER_OF_VARIABLES = 20000;
   private static final int BUFFER_SIZE = 8;
   private static final int WARMUP_ITERATIONS = 2000;
   private static final int TIMED_ITERATIONS = 5000;

   @Test
   public void benchmarkWriteBufferAt() throws IOException
   {
      Random random = new Random(1234L);

      YoRegistry rootRegistry = new YoRegistry("root");
      // Roughly mirrors a real robot registry: mostly doubles, with a modest mix of the other types.
      for (int i = 0; i < NUMBER_OF_VARIABLES; i++)
      {
         switch (i % 10)
         {
            case 0 -> SharedMemoryRandomTools.nextYoBoolean(random, "var" + i, rootRegistry);
            case 1 -> SharedMemoryRandomTools.nextYoInteger(random, "var" + i, rootRegistry);
            case 2 -> SharedMemoryRandomTools.nextYoEnum(random, "var" + i, rootRegistry);
            default -> SharedMemoryRandomTools.nextYoDouble(random, "var" + i, rootRegistry);
         }
      }

      YoBufferProperties bufferProperties = new YoBufferProperties(0, BUFFER_SIZE);
      YoRegistryBuffer registryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);
      List<YoVariable> allVariables = rootRegistry.collectSubtreeVariables();

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         allVariables.forEach(v -> SharedMemoryRandomTools.randomizeYoVariable(random, v));
         registryBuffer.writeBufferAt(i % BUFFER_SIZE);
      }

      long start = System.nanoTime();
      for (int i = 0; i < TIMED_ITERATIONS; i++)
      {
         registryBuffer.writeBufferAt(i % BUFFER_SIZE);
      }
      long elapsedNanos = System.nanoTime() - start;

      double avgCallMicros = elapsedNanos / 1000.0 / TIMED_ITERATIONS;
      String result = String.format("variables=%d availableProcessors=%d totalMs=%.2f avgCallMicros=%.3f%n",
                                     NUMBER_OF_VARIABLES,
                                     Runtime.getRuntime().availableProcessors(),
                                     elapsedNanos / 1e6,
                                     avgCallMicros);

      System.out.print("BENCHMARK writeBufferAt: " + result);

      Path resultsFile = Path.of("build", "benchmark", "writeBufferAt.txt");
      Files.createDirectories(resultsFile.getParent());
      Files.writeString(resultsFile, result, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
   }
}
