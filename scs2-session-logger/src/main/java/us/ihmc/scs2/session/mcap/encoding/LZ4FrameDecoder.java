package us.ihmc.scs2.session.mcap.encoding;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.lz4.LZ4FDecompressionContext;
import org.bytedeco.lz4.global.lz4;

import java.nio.ByteBuffer;

/**
 * Decodes LZ4 frame-format streams, backed by bytedeco's JNI bindings to the reference liblz4 implementation ({@code lz4frame.h}).
 * <p>
 * Because this calls into the real reference decompressor rather than a from-scratch Java port, it transparently supports frames written with
 * either independent or dependent (linked) blocks -- e.g. MCAP bags recorded by tools other than SCS2, such as {@code ros2 bag record}, which
 * default to linked blocks.
 * </p>
 */
public class LZ4FrameDecoder
{
   public byte[] decode(byte[] in, byte[] out)
   {
      return decode(in, 0, in.length, out, 0);
   }

   public byte[] decode(byte[] in, int inOffset, int inLength, byte[] out, int outOffset)
   {
      ByteBuffer inBuffer = ByteBuffer.wrap(in);
      ByteBuffer outBuffer = out == null ? null : ByteBuffer.wrap(out);
      ByteBuffer result = decode(inBuffer, inOffset, inLength, outBuffer, outOffset);
      return result.array();
   }

   public ByteBuffer decode(ByteBuffer in, ByteBuffer out)
   {
      return decode(in, 0, in.remaining(), out, 0);
   }

   public ByteBuffer decode(ByteBuffer in, int inOffset, int inLength, ByteBuffer out, int outOffset)
   {
      // Temporarily narrow the view to exactly [inOffset, inOffset + inLength) so decompress() only ever sees this one frame's bytes,
      // then restore the caller's original limit -- this method must not permanently mutate the buffer it was handed beyond position.
      int limitPrev = in.limit();
      in.position(inOffset);
      in.limit(inOffset + inLength);

      try
      {
         // Native calls need a real memory address, which only a direct buffer has; toDirect() copies in only if 'in' isn't already direct.
         ByteBuffer srcDirect = LZ4NativeUtil.toDirect(in);

         // We don't know the decompressed size up front (LZ4 frames don't have to declare it), so decompress() grows its own scratch buffer
         // as needed. When the caller gave us an output buffer we can at least size the first allocation to fit, avoiding a resize in the
         // common case; otherwise just guess generously off the compressed size.
         final int guessAtDecompressedMultiplier = 3; // Decompressed size might be 3x compressed size (just a guess)
         final int floorBufferSize = 64 * 1024; // This prevents starting with a really small buffer as our size guess
         int sizeHint = out != null ? out.remaining() : Math.max(inLength * guessAtDecompressedMultiplier, floorBufferSize);
         ByteBuffer decoded = decompress(srcDirect, sizeHint); // direct, flipped (position 0, limit = decoded length)

         if (out != null)
         {
            // Match the old decoder's contract: write into the caller's buffer starting at outOffset, then flip so the caller reads from 0.
            out.position(outOffset);
            out.put(decoded);
            out.flip();
            return out;
         }
         else
         {
            // No output buffer given: hand back a right-sized, heap-backed (non-direct) copy so callers can safely call ByteBuffer.array()
            // on it, matching what byte[]-based decode(...) overloads above expect.
            ByteBuffer result = ByteBuffer.allocate(decoded.remaining());
            result.put(decoded);
            result.flip();
            return result;
         }
      }
      finally
      {
         in.limit(limitPrev);
      }
   }

   /**
    * Runs the {@code LZ4F_decompress} streaming loop to completion. {@code srcDirect} must be a direct buffer positioned/limited to exactly the
    * compressed frame bytes; its position is advanced as it's consumed.
    * <p>
    * Note this decodes exactly one LZ4 frame: if {@code srcDirect} contains additional bytes after the first frame's end mark (e.g. multiple
    * concatenated frames), they are left unread. That's fine for MCAP, where a chunk's compressed payload is always exactly one frame, but it's a
    * capability the old lz4-java-derived decoder had (it looped to decode concatenated frames) that this one doesn't.
    * </p>
    *
    * @return a direct buffer, flipped, containing the decompressed bytes.
    */
   private static ByteBuffer decompress(ByteBuffer srcDirect, int initialCapacity)
   {
      // LZ4F_decompress is a streaming API: it's driven by repeatedly calling it, each call consuming some of the compressed input and
      // producing some decompressed output, until it reports the frame is fully decoded. The decompression context (dctx) is where it
      // keeps track of how far through the frame it is between calls.
      LZ4FDecompressionContext dctx = new LZ4FDecompressionContext();
      LZ4NativeUtil.checkError(lz4.LZ4F_createDecompressionContext(dctx, lz4.LZ4F_VERSION));

      try
      {
         final int OSMemoryPageSize = 4096; // Minimum buffer size for tons of I/O calls
         ByteBuffer destination = ByteBuffer.allocateDirect(Math.max(initialCapacity, OSMemoryPageSize));
         SizeTPointer destinationSize = new SizeTPointer(1); // in/out param: capacity offered in, bytes actually written out
         SizeTPointer sourceSize = new SizeTPointer(1); // in/out param: bytes offered in, bytes actually consumed out

         long ret;
         do
         {
            if (!destination.hasRemaining())
            {
               // Ran out of room before the frame finished decoding -- double the scratch buffer and keep going from where we left off.
               // This way we reach the end faster and faster if the destination is large
               ByteBuffer grown = ByteBuffer.allocateDirect(destination.capacity() * 2);
               destination.flip();
               grown.put(destination);
               destination = grown;
            }

            destinationSize.put(destination.remaining());
            sourceSize.put(srcDirect.remaining());

            Pointer dstPointer = new Pointer(destination);
            Pointer srcPointer = new Pointer(srcDirect);

            // ret == 0 means the frame is fully decoded. ret > 0 is a hint (not a byte count to rely on) that more calls are needed;
            // ret < 0 is only possible via the error encoding checked by checkError() below.
            ret = lz4.LZ4F_decompress(dctx, dstPointer, destinationSize, srcPointer, sourceSize, null);
            LZ4NativeUtil.checkError(ret);

            destination.position(destination.position() + (int) destinationSize.get());
            srcDirect.position(srcDirect.position() + (int) sourceSize.get());

            // If a call consumes no input and produces no output while still claiming the frame isn't finished, we're not going to make
            // any further progress -- almost always because the input was truncated/corrupt. Without this check we'd spin forever instead
            // of failing, since destination still has room and sourceSize would keep reporting 0 available bytes on every subsequent call.
            if (ret != 0 && destinationSize.get() == 0 && sourceSize.get() == 0)
               throw new IllegalStateException("Truncated or corrupt LZ4 frame: input ended before the frame finished decoding");
         }
         while (ret != 0);

         destination.flip();
         return destination;
      }
      finally
      {
         lz4.LZ4F_freeDecompressionContext(dctx);
      }
   }
}
