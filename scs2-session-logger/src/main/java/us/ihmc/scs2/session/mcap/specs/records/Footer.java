package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.util.Collection;

/**
 * Footer records contain end-of-file information. MCAP files must end with a Footer record.
 *
 * @see <a href="https://mcap.dev/spec#footer-op0x02">MCAP Footer</a>
 */
public class Footer implements MCAPElement
{
   public static final int ELEMENT_LENGTH = 2 * Long.BYTES + Integer.BYTES;
   private final MCAPDataInput dataInput;
   /**
    * Position in the file of the first record of the summary section.
    * <p>
    * The summary section contains schema, channel, chunk index, attachment index, metadata index, and statistics records.
    * It is not delimited by an encapsulating record as the rest, instead, the summary section simply starts right after the {@link DataEnd} record.
    * Note that the records in the summary section must be grouped by {@link Opcode}.
    * </p>
    */
   private final long summarySectionOffset;
   /**
    * The summary section is followed directly by summary offset records.
    * <p>
    * There is one summary offset record per {@link Opcode} in the summary section.
    * The summary offset records are used to quickly locate the start of each group.
    */
   private final long summaryOffsetSectionOffset;
   private final long summaryCRC32;
   private Records summarySection;
   private Records summaryOffsetSection;

   public Footer()
   {
      this.dataInput = null;
      this.summarySectionOffset = 0;
      this.summaryOffsetSectionOffset = 0;
      this.summaryCRC32 = 0;
   }

   public Footer(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;

      dataInput.position(elementPosition);
      summarySectionOffset = MCAP.checkPositiveLong(dataInput.getLong(), "summarySectionOffset");
      summaryOffsetSectionOffset = MCAP.checkPositiveLong(dataInput.getLong(), "summaryOffsetSectionOffset");
      summaryCRC32 = dataInput.getUnsignedInt();
      MCAP.checkLength(elementLength, getElementLength());
   }

   public Footer(long summarySectionOffset, Collection<? extends Record> summaryRecords, Collection<? extends Record> summaryOffsetRecords)
   {
      this.summarySection = new Records(summaryRecords);
      this.summaryOffsetSection = new Records(summaryOffsetRecords);

      this.summarySectionOffset = summarySectionOffset;
      this.summaryOffsetSectionOffset = summarySectionOffset + summarySection.getElementLength();

      MCAPCRC32Helper crc32 = new MCAPCRC32Helper();
      summarySection.forEach(record -> record.updateCRC(crc32));
      summaryOffsetSection.forEach(record -> record.updateCRC(crc32));
      crc32.addUnsignedByte(Opcode.FOOTER.id());
      crc32.addLong(ELEMENT_LENGTH);
      crc32.addLong(summarySectionOffset);
      crc32.addLong(summaryOffsetSectionOffset);
      this.summaryCRC32 = crc32.getValue();

      this.dataInput = null;
   }

   public static long computeOffsetFooter(MCAPDataInput dataInput)
   {
      // Offset to the beginning of the footer.
      return dataInput.size() - Record.RECORD_HEADER_LENGTH - ELEMENT_LENGTH - Magic.MAGIC_SIZE;
   }

   @Override
   public long getElementLength()
   {
      return ELEMENT_LENGTH;
   }

   public Records summarySection()
   {
      if (summarySection == null && summarySectionOffset != 0)
         summarySection = Records.load(dataInput, summarySectionOffset, summarySectionLength());
      return summarySection;
   }

   public long summarySectionLength()
   {
      long summarySectionEnd = summaryOffsetSectionOffset != 0 ? summaryOffsetSectionOffset : computeOffsetFooter(dataInput);
      return summarySectionEnd - summarySectionOffset;
   }

   public Records summaryOffsetSection()
   {
      if (summaryOffsetSection == null && summaryOffsetSectionOffset != 0)
         summaryOffsetSection = Records.load(dataInput, summaryOffsetSectionOffset, summaryOffsetSectionLength());
      return summaryOffsetSection;
   }

   public long summaryOffsetSectionLength()
   {
      return computeOffsetFooter(dataInput) - summaryOffsetSectionOffset;
   }

   private byte[] summaryCRC32Input;

   public byte[] summaryCRC32Input()
   {
      if (summaryCRC32Input == null)
      {
         long offset = summaryCRC32StartOffset();
         long length = dataInput.size() - offset - Magic.MAGIC_SIZE - Integer.BYTES;
         summaryCRC32Input = dataInput.getBytes(offset, (int) length);
      }
      return summaryCRC32Input;
   }

   /**
    * Offset to the first record of the summary section or to the footer if there is no summary section.
    * <p>
    * It is used to compute the CRC.
    * </p>
    */
   public long summaryCRC32StartOffset()
   {
      return summarySectionOffset != 0 ? summarySectionOffset : computeOffsetFooter(dataInput);
   }

   public long summarySectionOffset()
   {
      return summarySectionOffset;
   }

   public long summaryOffsetSectionOffset()
   {
      return summaryOffsetSectionOffset;
   }

   /**
    * A CRC-32 of all bytes from the start of the Summary section up through and including the end of
    * the previous field (summary_offset_start) in the footer record. A value of 0 indicates the CRC-32
    * is not available.
    */
   public long summaryCRC32()
   {
      return summaryCRC32;
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putLong(summarySectionOffset());
      dataOutput.putLong(summaryOffsetSectionOffset());
      dataOutput.putUnsignedInt(summaryCRC32());
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addLong(summarySectionOffset());
      crc32.addLong(summaryOffsetSectionOffset());
      crc32.addUnsignedInt(summaryCRC32());
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
      String out = getClass().getSimpleName() + ":";
      out += "\n\t-ofsSummarySection = " + summarySectionOffset();
      out += "\n\t-ofsSummaryOffsetSection = " + summaryOffsetSectionOffset();
      out += "\n\t-summaryCrc32 = " + summaryCRC32();
      return MCAPElement.indent(out, indent);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Footer other && equals(other);
   }

   @Override
   public boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof Footer other)
      {
         if (summarySectionOffset() != other.summarySectionOffset())
            return false;
         if (summaryOffsetSectionOffset() != other.summaryOffsetSectionOffset())
            return false;
         return summaryCRC32() == other.summaryCRC32();
      }

      return false;
   }
}
