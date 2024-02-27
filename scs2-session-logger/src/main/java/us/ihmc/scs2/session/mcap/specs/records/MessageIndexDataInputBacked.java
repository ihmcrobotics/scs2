package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;
import java.util.List;

public class MessageIndexDataInputBacked implements MessageIndex
{
   private final MCAPDataInput dataInput;
   private final long elementLength;
   private final int channelId;
   private WeakReference<List<MessageIndexEntry>> messageIndexEntriesRef;
   private final long messageIndexEntriesOffset;
   private final long messageIndexEntriesLength;

   public MessageIndexDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;
      this.elementLength = elementLength;

      dataInput.position(elementPosition);
      channelId = dataInput.getUnsignedShort();
      messageIndexEntriesLength = dataInput.getUnsignedInt();
      messageIndexEntriesOffset = dataInput.position();
   }

   @Override
   public long getElementLength()
   {
      return elementLength;
   }

   @Override
   public int channelId()
   {
      return channelId;
   }

   @Override
   public List<MessageIndexEntry> messageIndexEntries()
   {
      List<MessageIndexEntry> messageIndexEntries = messageIndexEntriesRef == null ? null : messageIndexEntriesRef.get();

      if (messageIndexEntries == null)
      {
         messageIndexEntries = MCAP.parseList(dataInput, MessageIndexEntry::new, messageIndexEntriesOffset, messageIndexEntriesLength);
         messageIndexEntriesRef = new WeakReference<>(messageIndexEntries);
      }

      return messageIndexEntries;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof MessageIndex other && MessageIndex.super.equals(other);
   }
}
