package us.ihmc.scs2.session.mcap.specs;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.records.Footer;
import us.ihmc.scs2.session.mcap.specs.records.MCAPElement;
import us.ihmc.scs2.session.mcap.specs.records.Magic;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.RecordDataInputBacked;

import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;

/**
 * MCAP is a modular container format and logging library for pub/sub messages with arbitrary
 * message serialization. It is primarily intended for use in robotics applications, and works well
 * under various workloads, resource constraints, and durability requirements. Time values
 * (`log_time`, `publish_time`, `create_time`) are represented in nanoseconds since a
 * user-understood epoch (i.e. Unix epoch, robot boot time, etc.)
 *
 * @see <a href="https://github.com/foxglove/mcap/tree/c1cc51d/docs/specification#readme">Source</a>
 */
public class MCAP
{
   /**
    * Stream object that this MCAP was parsed from.
    */
   protected MCAPDataInput dataInput;

   private final List<Record> records;

   private Record footer;

   public MCAP(FileChannel fileChannel)
   {
      dataInput = MCAPDataInput.wrap(fileChannel);

      long currentPos = 0;
      Magic.readMagic(dataInput, currentPos);
      currentPos += Magic.getElementLength();
      records = new ArrayList<>();
      Record lastRecord;

      try
      {
         do
         {
            lastRecord = new RecordDataInputBacked(dataInput, currentPos);
            if (lastRecord.getElementLength() < 0)
               throw new IllegalArgumentException("Invalid record length: " + lastRecord.getElementLength());
            currentPos += lastRecord.getElementLength();
            records.add(lastRecord);
         }
         while (!(lastRecord.op() == Opcode.FOOTER));
      }
      catch (IllegalArgumentException e)
      {
         try
         {

            LogTools.info("Loaded records:\n");
            for (Record record : records)
            {
               System.out.println(record);
            }
         }
         catch (Exception e2)
         {
            throw e;
         }
         throw e;
      }

      Magic.readMagic(dataInput, currentPos);
   }

   public MCAPDataInput getDataInput()
   {
      return dataInput;
   }

   public List<Record> records()
   {
      return records;
   }

   public Record footer()
   {
      if (footer == null)
      {
         footer = new RecordDataInputBacked(dataInput, Footer.computeOffsetFooter(dataInput));
      }
      return footer;
   }

   public static <T extends MCAPElement> List<T> parseList(MCAPDataInput dataInput, MCAPDataReader<T> elementParser)
   {
      return parseList(dataInput, elementParser, dataInput.getUnsignedInt());
   }

   public static <T extends MCAPElement> List<T> parseList(MCAPDataInput dataInput, MCAPDataReader<T> elementParser, long length)
   {
      return parseList(dataInput, elementParser, dataInput.position(), length);
   }

   public static <T extends MCAPElement> List<T> parseList(MCAPDataInput dataInput, MCAPDataReader<T> elementParser, long offset, long length)
   {
      return parseList(dataInput, elementParser, offset, length, null);
   }

   public static <T extends MCAPElement> List<T> parseList(MCAPDataInput dataInput,
                                                           MCAPDataReader<T> elementParser,
                                                           long offset,
                                                           long length,
                                                           List<T> listToPack)
   {
      long position = offset;
      long limit = position + length;
      if (listToPack == null)
         listToPack = new ArrayList<>();

      while (position < limit)
      {
         T parsed = elementParser.parse(dataInput, position);
         listToPack.add(parsed);
         position += parsed.getElementLength();
      }

      return listToPack;
   }

   public interface MCAPDataReader<T extends MCAPElement>
   {
      T parse(MCAPDataInput dataInput, long position);
   }

   public static long checkPositiveLong(long value, String name)
   {
      if (value < 0)
         throw new IllegalArgumentException(name + " must be positive. Value: " + value);
      return value;
   }

   public static void checkLength(long expectedLength, long actualLength)
   {
      if (actualLength != expectedLength)
         throw new IllegalArgumentException("Unexpected length: expected= " + expectedLength + ", actual= " + actualLength);
   }
}