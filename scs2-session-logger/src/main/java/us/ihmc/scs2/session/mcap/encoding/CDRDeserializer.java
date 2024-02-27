package us.ihmc.scs2.session.mcap.encoding;

import us.ihmc.idl.CDR;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.common.SerializedPayload;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Helper class to deserialize data from a buffer that uses the CDR (Common Data Representation) format.
 */
public class CDRDeserializer
{
   private static final int encapsulation_size = 4;
   private ByteBuffer buffer;
   private int initialOffset;
   private int initialLimit;
   private int offset;

   public enum Type
   {
      BOOL(1, -1),
      BOOLEAN(1, -1),
      FLOAT(Float.BYTES, 4),
      FLOAT32(Float.BYTES, 4),
      DOUBLE(Double.BYTES, 8),
      FLOAT64(Double.BYTES, 8),
      CHAR(Byte.BYTES, -1),
      OCTET(Byte.BYTES, -1),
      BYTE(Byte.BYTES, -1),
      INT8(Byte.BYTES, -1),
      UINT8(Byte.BYTES, -1),
      SHORT(Short.BYTES, 2),
      INT16(Short.BYTES, 2),
      UNSIGNEDSHORT(Short.BYTES, 2),
      UINT16(Short.BYTES, 2),
      LONG(Integer.BYTES, 4),
      INT32(Integer.BYTES, 4),
      UNSIGNEDLONG(Integer.BYTES, 4),
      UINT32(Integer.BYTES, 4),
      LONGLONG(Long.BYTES, 8),
      INT64(Long.BYTES, 8),
      UNSIGNEDLONGLONG(Long.BYTES, 8),
      UINT64(Long.BYTES, 8),
      STRING(-1, -1),
      ARRAY(-1, -1),
      SEQUENCE(-1, -1);

      private final int bytes;
      private final int byteBoundary;

      Type(int bytes, int byteBoundary)
      {
         this.bytes = bytes;
         this.byteBoundary = byteBoundary;
      }

      public static Type parseType(String stringValue)
      {
         for (Type type : values())
         {
            if (type.name().equalsIgnoreCase(stringValue))
               return type;
         }
         throw new IllegalArgumentException("Unexpected type: " + stringValue);
      }
   }

   /**
    * Initializes this deserializer to read from the given buffer.
    *
    * @param buffer the buffer to read from.
    */
   public void initialize(ByteBuffer buffer)
   {
      initialize(buffer, -1, -1);
   }

   /**
    * Initializes this deserializer to read from the given buffer.
    *
    * @param buffer the buffer to read from.
    * @param offset the offset from which to start reading.
    * @param length the length of the buffer to read.
    */
   public void initialize(ByteBuffer buffer, int offset, int length)
   {
      this.initialOffset = buffer.position();
      this.initialLimit = buffer.limit();

      this.offset = offset > -1 ? offset : buffer.position();
      if (length > -1)
         buffer.limit(this.offset + length);
      if (offset > -1)
         buffer.position(offset);

      readEncapsulation(buffer);
      this.buffer = buffer;
   }

   /**
    * Finalizes this deserializer.
    *
    * @param resetBufferPosition whether to reset the buffer to its initial position.
    */
   public void finalize(boolean resetBufferPosition)
   {
      if (resetBufferPosition)
      {
         if (initialOffset > buffer.limit())
            buffer.limit(initialLimit);
         buffer.position(initialOffset);
      }
      buffer.limit(initialLimit);
   }

   /**
    * Reads the encapsulation from the buffer.
    *
    * @param buffer the buffer to read from.
    */
   private static void readEncapsulation(ByteBuffer buffer)
   {
      // @formatter:off
      /* int dummy = */ buffer.get();
      short encapsulation = buffer.get();
      if (encapsulation == SerializedPayload.CDR_BE || encapsulation == SerializedPayload.PL_CDR_BE)
      {
         buffer.order(ByteOrder.BIG_ENDIAN);
      }
      else
      {
         buffer.order(ByteOrder.LITTLE_ENDIAN);
      }
      /* this.options = */ buffer.getShort();
      // @formatter:on
   }

   /**
    * Reads a boolean {@code bool} from the buffer.
    *
    * @return the boolean value.
    * @see CDR#read_type_7()
    */
   public boolean read_bool()
   {
      return buffer.get() != (byte) 0;
   }

   /**
    * Reads a float {@code float32} from the buffer.
    *
    * @return the float value.
    * @see CDR#read_type_5()
    */
   public float read_float32()
   {
      align(Type.FLOAT32.byteBoundary);
      return buffer.getFloat();
   }

   /**
    * Reads a double {@code float64} from the buffer.
    *
    * @return the double value.
    * @see CDR#read_type_6()
    */
   public double read_float64()
   {
      align(Type.FLOAT64.byteBoundary);
      return buffer.getDouble();
   }

   /**
    * Reads a byte {@code byte} from the buffer.
    *
    * @return the byte value.
    * @see CDR#read_type_9()
    */
   public byte read_byte()
   {
      return buffer.get();
   }

   /**
    * Reads a byte {@code int8} from the buffer.
    *
    * @return the byte value.
    * @see CDR#read_type_9()
    */
   public byte read_int8()
   {
      return read_byte();
   }

   /**
    * Reads an unsigned byte {@code uint8} from the buffer.
    *
    * @return the unsigned byte value.
    * @see CDR#read_type_9()
    */
   public int read_uint8()
   {
      return Byte.toUnsignedInt(read_int8());
   }

   /**
    * Reads a short {@code int16} from the buffer.
    *
    * @return the short value.
    * @see CDR#read_type_1()
    */
   public short read_int16()
   {
      align(Type.INT16.byteBoundary);
      return buffer.getShort();
   }

   /**
    * Reads an unsigned short {@code uint16} from the buffer.
    *
    * @return the unsigned short value.
    * @see CDR#read_type_3()
    */
   public int read_uint16()
   {
      return Short.toUnsignedInt(read_int16());
   }

   /**
    * Reads an integer {@code int32} from the buffer.
    *
    * @return the integer value.
    * @see CDR#read_type_2()
    */
   public int read_int32()
   {
      align(Type.INT32.byteBoundary);
      return buffer.getInt();
   }

   /**
    * Reads an unsigned integer {@code uint32} from the buffer.
    *
    * @return the unsigned integer value.
    * @see CDR#read_type_4()
    */
   public long read_uint32()
   {
      return Integer.toUnsignedLong(read_int32());
   }

   /**
    * Reads a long {@code int64} from the buffer.
    *
    * @return the long value.
    * @see CDR#read_type_11()
    */
   public long read_int64()
   {
      align(Type.INT64.byteBoundary);
      return buffer.getLong();
   }

   /**
    * Reads an unsigned long {@code uint64} from the buffer.
    * <p>
    * Note that there is no unsigned long in Java, so the value is returned as a long.
    * The value is checked to be positive and a warning is printed if it is not.
    * </p>
    *
    * @return the unsigned long value.
    * @see CDR#read_type_12()
    */
   public long read_uint64()
   {
      // No unsigned long in Java
      long uint64 = read_int64();
      if (uint64 < 0)
         LogTools.warn("uint64 value is negative: " + uint64);
      return uint64;
   }

   /**
    * Reads a string {@code string} from the buffer.
    *
    * @return the string value.
    * @see CDR#read_type_d(StringBuilder)
    */
   public String read_string()
   {
      return new String(read_stringAsBytes());
   }

   /**
    * Reads a string {@code string} from the buffer and returns the result as a byte array for each character.
    *
    * @return the string value as a byte array.
    * @see CDR#read_type_d(StringBuilder)
    */
   public byte[] read_stringAsBytes()
   {
      int length = read_int32() - 1;
      byte[] bytes = new byte[length];
      buffer.get(bytes);
      buffer.get(); // Discard the null terminator
      return bytes;
   }

   /**
    * Reads a string {@code string} from the buffer into the given {@code stringBuilderToPack}.
    *
    * @param stringBuilderToPack the string builder to which the characters are added.
    * @see CDR#read_type_d(StringBuilder)
    */
   public void read_string(StringBuilder stringBuilderToPack)
   {
      int length = read_int32() - 1;

      for (int i = 0; i < length; i++)
      {
         stringBuilderToPack.append((char) buffer.get());
      }
      buffer.get(); // Discard the null terminator
   }

   /**
    * Reads a fixed-size array from the buffer.
    * <p>
    * Both a reader and the size of the array need to be provided for this type.
    * </p>
    *
    * @param reader      the reader to use to read the array elements.
    * @param arrayLength the size of the array to read.
    * @see CDR#read_type_f()
    */
   public void read_array(ElementReader reader, int arrayLength)
   {
      for (int i = 0; i < arrayLength; i++)
      {
         reader.read(i, this);
      }
   }

   /**
    * Reads a sequence from the buffer. The difference with an array is that the size of the sequence is not known in advance, only a maximum size is known.
    *
    * @param reader the reader to use to read the sequence elements.
    * @return the actual size of the sequence.
    */
   public int read_sequence(ElementReader reader)
   {
      int length = read_int32();

      for (int i = 0; i < length; i++)
      {
         reader.read(i, this);
      }

      return length;
   }

   /**
    * Reads a type from the buffer and returns its value as a double.
    *
    * @param type the type to read.
    * @return the value of the type as a double.
    */
   public double readTypeAsDouble(Type type)
   {
      return switch (type)
      {
         case BOOL, BOOLEAN -> read_bool() ? 1.0 : 0.0;
         case FLOAT, FLOAT32 -> read_float32();
         case DOUBLE, FLOAT64 -> read_float64();
         case CHAR, OCTET, BYTE, INT8 -> read_int8();
         case UINT8 -> read_uint8();
         case SHORT, INT16 -> read_int16();
         case UNSIGNEDSHORT, UINT16 -> read_uint16();
         case LONG, INT32 -> read_int32();
         case UNSIGNEDLONG, UINT32 -> read_uint32();
         case LONGLONG, INT64 -> read_int64();
         case UNSIGNEDLONGLONG, UINT64 -> read_uint64();
         default -> throw new IllegalArgumentException("Unexpected type: " + type);
      };
   }

   /**
    * Reads a type from the buffer and returns its value as a string.
    *
    * @param type the type to read.
    * @return the value of the type as a string.
    */
   public String readTypeAsString(Type type)
   {
      return readTypeAsString(type, -1);
   }

   /**
    * Reads a type from the buffer and returns its value as a string.
    *
    * @param type        the type to read.
    * @param arrayLength the length of the array to read if known, {@code -1} if unknown.
    * @return the value of the type as a string.
    */
   public String readTypeAsString(Type type, int arrayLength)
   {
      return switch (type)
      {
         case BOOL, BOOLEAN -> String.valueOf(read_bool());
         case FLOAT, FLOAT32 -> String.valueOf(read_float32());
         case DOUBLE, FLOAT64 -> String.valueOf(read_float64());
         case CHAR, OCTET, BYTE, INT8 -> String.valueOf(read_int8());
         case UINT8 -> String.valueOf(read_uint8());
         case SHORT, INT16 -> String.valueOf(read_int16());
         case UNSIGNEDSHORT, UINT16 -> String.valueOf(read_uint16());
         case LONG, INT32 -> String.valueOf(read_int32());
         case UNSIGNEDLONG, UINT32 -> String.valueOf(read_uint32());
         case LONGLONG, INT64 -> String.valueOf(read_int64());
         case UNSIGNEDLONGLONG, UINT64 -> String.valueOf(read_uint64());
         case STRING -> read_string();
         case ARRAY ->
         {
            if (arrayLength < 0)
               yield null;

            StringBuilder sb = new StringBuilder();
            sb.append("[");
            for (int i = 0; i < arrayLength; i++)
            {
               sb.append(readTypeAsString(type, -1));
               if (i < arrayLength - 1)
                  sb.append(", ");
            }
            sb.append("]");
            yield sb.toString();
         }
         case SEQUENCE ->
         {
            StringBuilder sb = new StringBuilder();
            sb.append("[");
            int length = read_int32();
            for (int i = 0; i < length; i++)
            {
               sb.append(readTypeAsString(type, -1));
               if (i < length - 1)
                  sb.append(", ");
            }
            sb.append("]");
            yield sb.toString();
         }
      };
   }

   /**
    * Skips the next element in the buffer.
    * <p>
    * Note that this method does not support skipping arrays.
    * </p>
    *
    * @param nextType the type of the next element to skip.
    * @return {@code true} if the element was skipped, {@code false} if the element could not be skipped.
    */
   public boolean skipNext(Type nextType)
   {
      return skipNext(nextType, -1);
   }

   /**
    * Skips the next element in the buffer.
    *
    * @param nextType    the type of the next element to skip.
    * @param arrayLength the length of the array to skip if known, {@code -1} if unknown.
    * @return {@code true} if the element was skipped, {@code false} if the element could not be skipped.
    */
   public boolean skipNext(Type nextType, int arrayLength)
   {
      boolean success = true;
      if (nextType.bytes > -1)
      {
         align(nextType.byteBoundary);
         buffer.position(buffer.position() + nextType.bytes);
      }
      else
      {
         switch (nextType)
         {
            case STRING ->
            {
               int length = read_int32() - 1;
               buffer.position(buffer.position() + length + 1);
            }
            case ARRAY ->
            {
               if (arrayLength < 0)
                  success = false;
               else
                  buffer.position(buffer.position() + arrayLength);
            }
            case SEQUENCE ->
            {
               int length = read_int32();
               buffer.position(buffer.position() + length);
            }
         }
      }
      return success;
   }

   private int align(int byteBoundary)
   {
      int position = buffer.position() - offset - encapsulation_size;
      int adv = (position % byteBoundary);

      if (adv != 0)
      {
         buffer.position(position + offset + encapsulation_size + (byteBoundary - adv));
      }

      return adv;
   }

   /**
    * Interface used to read elements of an array or sequence from a buffer.
    */
   public interface ElementReader
   {
      /**
       * Reads an element of an array or a sequence from the buffer.
       *
       * @param elementIndex the index of the element to read.
       * @param deserializer the deserializer to use to read the element.
       */
      void read(int elementIndex, CDRDeserializer deserializer);
   }
}
