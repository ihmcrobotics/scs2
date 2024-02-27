package us.ihmc.scs2.session.mcap.specs.records;

import java.util.ArrayList;
import java.util.List;

public class MutableMessageIndex implements MessageIndex
{
   private int channelId;
   private List<MessageIndexEntry> messageIndexEntries;
   private long messageIndexEntriesLength = 0;

   public void setChannelId(int channelId)
   {
      this.channelId = channelId;
   }

   public void setMessageIndexEntries(List<MessageIndexEntry> messageIndexEntries)
   {
      this.messageIndexEntries = messageIndexEntries;
      messageIndexEntriesLength = messageIndexEntries.stream().mapToLong(MessageIndexEntry::getElementLength).sum();
   }

   public void addMessageIndexEntry(MessageIndexEntry messageIndexEntry)
   {
      if (messageIndexEntries == null)
         messageIndexEntries = new ArrayList<>();
      messageIndexEntries.add(messageIndexEntry);
      messageIndexEntriesLength += messageIndexEntry.getElementLength();
   }

   public long messageIndexEntriesLength()
   {
      return messageIndexEntriesLength;
   }

   @Override
   public long getElementLength()
   {
      return Short.BYTES + Integer.BYTES + messageIndexEntriesLength;
   }

   @Override
   public int channelId()
   {
      return channelId;
   }

   @Override
   public List<MessageIndexEntry> messageIndexEntries()
   {
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
