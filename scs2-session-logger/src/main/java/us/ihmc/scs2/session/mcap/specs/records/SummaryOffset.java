package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.Reference;
import java.lang.ref.SoftReference;
import java.lang.ref.WeakReference;
import java.util.List;

/**
 * A Summary Offset record contains the location of records within the summary section.
 * Each Summary Offset record corresponds to a group of summary records with the same opcode.
 *
 * @see <a href="https://mcap.dev/spec#summary-offset-op0x0e">MCAP Summary Offset</a>
 */
public class SummaryOffset implements MCAPElement
{
   int ELEMENT_LENGTH = Byte.BYTES + 2 * Long.BYTES;

   private final MCAPDataInput dataInput;
   private final Opcode groupOpcode;
   private final long groupOffset;
   private final long groupLength;

   private Reference<Records> groupRef;

   public SummaryOffset(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;

      dataInput.position(elementPosition);
      groupOpcode = Opcode.byId(dataInput.getUnsignedByte());
      groupOffset = MCAP.checkPositiveLong(dataInput.getLong(), "offsetGroup");
      groupLength = MCAP.checkPositiveLong(dataInput.getLong(), "lengthGroup");
      MCAP.checkLength(elementLength, getElementLength());
   }

   public SummaryOffset(Opcode groupOpcode, long groupOffset, long groupLength)
   {
      this.dataInput = null;
      this.groupOpcode = groupOpcode;
      this.groupOffset = groupOffset;
      this.groupLength = groupLength;
   }

   public SummaryOffset(long groupOffset, List<Record> recordGroup)
   {
      if (recordGroup.isEmpty())
         throw new IllegalArgumentException("The record group cannot be empty");

      groupOpcode = recordGroup.get(0).op();

      if (recordGroup.stream().anyMatch(record -> record.op() != groupOpcode))
         throw new IllegalArgumentException("All records in the group must have the same opcode");

      this.groupOffset = groupOffset;
      groupLength = recordGroup.stream().mapToLong(Record::getElementLength).sum();

      groupRef = new SoftReference<>(new Records(recordGroup));
      dataInput = null;
   }

   @Override
   public long getElementLength()
   {
      return ELEMENT_LENGTH;
   }

   public Records group()
   {
      Records group = groupRef == null ? null : groupRef.get();

      if (group == null)
      {
         if (dataInput == null)
            throw new IllegalStateException("This record is not backed by a data input.");

         group = Records.load(dataInput, groupOffset, (int) groupLength);
         groupRef = new WeakReference<>(group);
      }
      return group;
   }

   public Opcode groupOpcode()
   {
      return groupOpcode;
   }

   public long groupOffset()
   {
      return groupOffset;
   }

   public long groupLength()
   {
      return groupLength;
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putUnsignedByte(groupOpcode().id());
      dataOutput.putLong(groupOffset());
      dataOutput.putLong(groupLength());
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addUnsignedByte(groupOpcode().id());
      crc32.addLong(groupOffset());
      crc32.addLong(groupLength());
      return crc32;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public String toString(int indent)
   {
      String out = getClass().getSimpleName() + ": ";
      out += "\n\t-groupOpcode = " + groupOpcode();
      out += "\n\t-groupOffset = " + groupOffset();
      out += "\n\t-groupLength = " + groupLength();
      return MCAPElement.indent(out, indent);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof SummaryOffset other && equals(other);
   }

   @Override
   public boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof SummaryOffset other)
      {
         if (groupOpcode() != other.groupOpcode())
            return false;
         if (groupOffset() != other.groupOffset())
            return false;
         return groupLength() == other.groupLength();
      }

      return false;
   }
}
