package us.ihmc.scs2.session.mcap.encoding;

import org.bytedeco.lz4.global.lz4;

import java.nio.ByteBuffer;

/**
 * Shared helpers for bridging {@link ByteBuffer}s to native memory for {@link LZ4FrameDecoder} and {@link LZ4FrameEncoder}, both of which are
 * backed by bytedeco's JNI bindings to the reference liblz4 implementation (the real {@code lz4frame.h} API, not a Java reimplementation).
 */
class LZ4NativeUtil
{
   private LZ4NativeUtil()
   {
   }

   /**
    * Native calls need a direct buffer to obtain a native pointer from. Returns the buffer as-is if it's already direct (zero-copy), otherwise
    * copies its remaining bytes into a new direct buffer.
    */
   static ByteBuffer toDirect(ByteBuffer buffer)
   {
      if (buffer.isDirect())
         return buffer;
      ByteBuffer direct = ByteBuffer.allocateDirect(buffer.remaining());
      direct.put(buffer.duplicate());
      direct.flip();
      return direct;
   }

   static void checkError(long code)
   {
      if (lz4.LZ4F_isError(code) != 0)
         throw new RuntimeException(lz4.LZ4F_getErrorName(code).getString());
   }
}
