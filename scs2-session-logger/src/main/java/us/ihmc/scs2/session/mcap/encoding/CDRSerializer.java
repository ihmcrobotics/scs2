package us.ihmc.scs2.session.mcap.encoding;

import us.ihmc.idl.CDR;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer.Type;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import static us.ihmc.scs2.session.mcap.encoding.CDRDeserializer.encapsulation_size;

public class CDRSerializer
{
   private ByteBuffer buffer;

   /**
    * Initializes this serializer to write into the given buffer.
    *
    * @param buffer the buffer to write into.
    */
   public void initialize(ByteBuffer buffer)
   {
      short encapsulation;
      if (buffer.order() == ByteOrder.LITTLE_ENDIAN)
         encapsulation = SerializedPayload.CDR_LE;
      else
         encapsulation = SerializedPayload.CDR_BE;

      short options = 0x0;
      writeEncapsulation(buffer, encapsulation, options);

      this.buffer = buffer;
   }

   private static void writeEncapsulation(ByteBuffer buffer, short encapsulation, short options)
   {
      buffer.put((byte) 0x00);
      buffer.put((byte) encapsulation);
      buffer.putShort(options);
   }

   /**
    * Writes a boolean {@code bool} into the buffer.
    *
    * @param value the boolean value.
    * @see CDR#write_type_7(boolean)
    */
   public void write_bool(boolean value)
   {
      buffer.put(value ? (byte) 1 : (byte) 0);
   }

   /**
    * Writes a float {@code float32} into the buffer.
    *
    * @param value the float value.
    * @see CDR#write_type_5(float)
    */
   public float read_float32(float value)
   {
      align(Type.FLOAT32.byteBoundary);
      return buffer.getFloat();
   }

   /**
    * Writes a double {@code float64} into the buffer.
    *
    * @param value the double value.
    * @see CDR#write_type_6(double)
    */
   public void write_float64(double value)
   {
      align(Type.FLOAT64.byteBoundary);
      buffer.putDouble(value);
   }

   /**
    * Writes a byte {@code byteValue} into the buffer.
    *
    * @param byteValue the byte value.
    * @see CDR#write_type_9(byte)
    */
   public void write_byte(byte byteValue)
   {
      buffer.put(byteValue);
   }

   /**
    * Writes a byte {@code int8} into the buffer.
    *
    * @param value the byte value.
    * @see CDR#write_type_9(byte)
    */
   public void write_int8(byte value)
   {
      write_byte(value);
   }

   /**
    * Writes an unsigned byte {@code uint8} into the buffer.
    *
    * @param value the unsigned byte value.
    * @see CDR#write_type_9(byte)
    */
   public void write_uint8(int value)
   {
      write_byte((byte) value);
   }

   /**
    * Writes a short {@code int16} into the buffer.
    *
    * @param value the short value.
    * @see CDR#write_type_1(short)
    */
   public void write_int16(short value)
   {
      align(Type.INT16.byteBoundary);
      buffer.putShort(value);
   }

   /**
    * Writes an unsigned short {@code uint16} into the buffer.
    *
    * @param value the unsigned short value.
    * @see CDR#write_type_3(int)
    */
   public void write_uint16(int value)
   {
      if (value < 0 || value > 0xFFFF)
         throw new IllegalArgumentException("Value is out of range for uint16: " + value);
      write_int16((short) value);
   }

   /**
    * Writes an int {@code int32} into the buffer.
    *
    * @param value the int value.
    * @see CDR#write_type_2(int)
    */
   public void write_int32(int value)
   {
      align(Type.INT32.byteBoundary);
      buffer.putInt(value);
   }

   /**
    * Writes an unsigned int {@code uint32} into the buffer.
    *
    * @param value the unsigned int value.
    * @see CDR#write_type_4(long)
    */
   public void write_uint32(long value)
   {
      if (value < 0 || value > 0xFFFFFFFFL)
         throw new IllegalArgumentException("Value is out of range for uint32: " + value);
      write_int32((int) value);
   }

   /**
    * Writes a long {@code int64} into the buffer.
    *
    * @param value the long value.
    * @see CDR#write_type_11(long)
    */
   public void write_int64(long value)
   {
      align(Type.INT64.byteBoundary);
      buffer.putLong(value);
   }

   /**
    * Writes an unsigned long {@code uint64} into the buffer.
    *
    * @param value the unsigned long value.
    * @see CDR#write_type_12(long)
    */
   public void write_uint64(long value)
   {
      write_int64(value);
   }

   /**
    * Writes a string {@code string} into the buffer.
    *
    * @param value the string value.
    * @see CDR#write_type_d(StringBuilder)
    */
   public void write_string(String value)
   {
      if (value == null)
         throw new IllegalArgumentException("String cannot be null");

      write_uint32(value.length() + 1);
      for (int i = 0; i < value.length(); i++)
         buffer.put((byte) value.charAt(i));
      buffer.put((byte) 0);
   }

   /**
    * Writes a string {@code string} into the buffer.
    *
    * @param value the string value.
    * @see CDR#write_type_d(StringBuilder)
    */
   public void write_string(StringBuilder value)
   {
      if (value == null)
         throw new IllegalArgumentException("String cannot be null");

      write_uint32(value.length() + 1);
      for (int i = 0; i < value.length(); i++)
         buffer.put((byte) value.charAt(i));
      buffer.put((byte) 0);
   }

   /**
    * Writes a fixed-size array into the buffer.
    * <p>
    * The difference
    * </p>
    *
    * @param elementWriter the writer to use to write the elements.
    * @param arrayLength   the size of the array to write.
    */
   public void write_array(ElementWriter elementWriter, int arrayLength)
   {
      for (int i = 0; i < arrayLength; i++)
      {
         elementWriter.write(i, this);
      }
   }

   /**
    * Writes a sequence of elements into the buffer.
    *
    * @param length        the number of elements in the sequence.
    * @param elementWriter the writer to use to write the elements.
    */
   public void write_sequence(int length, ElementWriter elementWriter)
   {
      write_uint32(length);
      for (int i = 0; i < length; i++)
         elementWriter.write(i, this);
   }

   private int align(int byteBoundary)
   {
      int position = buffer.position() - encapsulation_size;
      int adv = (position % byteBoundary);

      if (adv != 0)
      {
         buffer.position(position + encapsulation_size + (byteBoundary - adv));
      }

      return adv;
   }

   /**
    * Interface used to write elements of an array or sequence into a buffer.
    */
   public interface ElementWriter
   {
      /**
       * Writes an element of an array or a sequence into the buffer.
       *
       * @param elementIndex the index of the element to write.
       * @param serializer   the serializer to use to write the element.
       */
      void write(int elementIndex, CDRSerializer serializer);
   }
}
