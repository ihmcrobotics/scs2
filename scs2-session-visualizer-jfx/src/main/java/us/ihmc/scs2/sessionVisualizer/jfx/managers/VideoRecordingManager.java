package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.embed.swing.SwingFXUtils;
import javafx.geometry.Rectangle2D;
import javafx.scene.Group;
import javafx.scene.SnapshotParameters;
import javafx.scene.SubScene;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import us.ihmc.codecs.builder.H264Settings;
import us.ihmc.codecs.builder.MP4H264MovieBuilder;
import us.ihmc.codecs.generated.EProfileIdc;
import us.ihmc.codecs.generated.EUsageType;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SceneVideoRecordingRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.BufferedJavaFXMessager;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

public class VideoRecordingManager
{
   private final SubScene scene;
   private final Group mainView3DRoot;
   private final SessionVisualizerTopics topics;
   private final BufferedJavaFXMessager messager;

   private final AtomicReference<YoBufferPropertiesReadOnly> currentBufferProperties;
   private final AtomicReference<SessionMode> currentSessionMode;
   private final AtomicReference<Long> sessionDT;
   private final AtomicReference<Integer> bufferRecordTickPeriod;

   private final AtomicReference<Recorder> activeRecorder = new AtomicReference<>(null);
   private final BackgroundExecutorManager backgroundExecutorManager;

   public VideoRecordingManager(SubScene scene,
                                Group mainView3DRoot,
                                SessionVisualizerTopics topics,
                                BufferedJavaFXMessager messager,
                                BackgroundExecutorManager backgroundExecutorManager)
   {
      this.scene = scene;
      this.mainView3DRoot = mainView3DRoot;
      this.messager = messager;
      this.topics = topics;
      this.backgroundExecutorManager = backgroundExecutorManager;

      currentBufferProperties = messager.createInput(topics.getYoBufferCurrentProperties());
      currentSessionMode = messager.createInput(topics.getSessionCurrentMode());
      sessionDT = messager.createInput(topics.getSessionDTNanoseconds());
      bufferRecordTickPeriod = messager.createInput(topics.getBufferRecordTickPeriod());

      messager.registerTopicListener(topics.getSceneVideoRecordingRequest(), request -> submitRequest(request));
   }

   private void submitRequest(SceneVideoRecordingRequest request)
   {
      LogTools.info("Received video export request: " + request);

      if (activeRecorder.get() != null)
         activeRecorder.get().stop();

      SnapshotParameters params = new SnapshotParameters();
      params.setCamera(scene.getCamera());
      params.setViewport(new Rectangle2D(0, 0, request.getWidth(), request.getHeight()));
      params.setDepthBuffer(true);
      params.setFill(Color.GRAY);

      Recorder recorder = new Recorder(request, params, () ->
      {
         activeRecorder.set(null);
         try
         {
            if (request.getRecordingEndedCallback() != null)
               request.getRecordingEndedCallback().run();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
         messager.submitMessage(topics.getDisableUserControls(), false);
      });
      activeRecorder.set(recorder);
      messager.submitMessage(topics.getDisableUserControls(), true);

      if (request.getRecordingStartedCallback() != null)
         request.getRecordingStartedCallback().run();

      recorder.start();
   }

   private class Recorder extends AnimationTimer
   {
      private final SceneVideoRecordingRequest request;

      private int inPoint = -1;
      private int outPoint = -1;

      private int currentRecordingBufferIndex;
      private int bufferIndexIncrement = -1;
      private int numberOfBufferTicks = -1;

      private int currentPhase = 0;

      private final SnapshotParameters params;

      private final Runnable stopListener;

      private boolean isDoneTakingJavaFXSnapshots = false;
      private boolean isDoneExporting = false;
      private final ConcurrentLinkedQueue<WritableImage> jfxImageQueue = new ConcurrentLinkedQueue<>();

      public Recorder(SceneVideoRecordingRequest request, SnapshotParameters params, Runnable stopListener)
      {
         this.request = request;
         this.params = params;
         this.stopListener = stopListener;

         H264Settings settings = new H264Settings();
         settings.setBitrate(request.getWidth() * request.getHeight() / 100);
         settings.setUsageType(EUsageType.CAMERA_VIDEO_REAL_TIME);
         settings.setProfileIdc(EProfileIdc.PRO_HIGH);

         backgroundExecutorManager.executeInBackground(new Runnable()
         {
            MP4H264MovieBuilder movieBuilder;
            BufferedImage bufferedImage = new BufferedImage(request.getWidth(), request.getHeight(), BufferedImage.TYPE_INT_ARGB);

            @Override
            public void run()
            {
               try
               {
                  if (!request.getFile().exists())
                  {
                     File parent = request.getFile().getCanonicalFile().getParentFile();

                     if (parent != null && !parent.mkdirs() && !parent.isDirectory())
                     {
                        throw new IOException("Unable to create parent directory of " + request.getFile());
                     }
                  }

                  if (!request.getFile().getName().toLowerCase().endsWith(SessionVisualizerIOTools.videoFileExtension))
                  {
                     LogTools.warn("Improper filename: {}, expected to end with the filename extension: \"{}\"",
                                   request.getFile().getName(),
                                   SessionVisualizerIOTools.videoFileExtension);
                  }

                  movieBuilder = new MP4H264MovieBuilder(request.getFile(), request.getWidth(), request.getHeight(), (int) request.getFrameRate(), settings);
               }
               catch (IOException e)
               {
                  e.printStackTrace();
                  return;
               }

               while (true)
               {
                  if (isDoneTakingJavaFXSnapshots && jfxImageQueue.isEmpty())
                     break;

                  if (!jfxImageQueue.isEmpty())
                     encodeNextFrame(jfxImageQueue.poll());
               }

               try
               {
                  movieBuilder.close();
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
               finally
               {
                  isDoneExporting = true;
               }
            }

            private void encodeNextFrame(WritableImage jfxImageToEncode)
            {
               if (jfxImageToEncode == null)
                  return;

               bufferedImage = SwingFXUtils.fromFXImage(jfxImageToEncode, bufferedImage);

               try
               {
                  movieBuilder.encodeFrame(bufferedImage);
               }
               catch (IOException e)
               {
                  e.printStackTrace();
                  stop();
               }
            }
         });
      }

      @Override
      public void handle(long now)
      {
         if (currentSessionMode.get() != SessionMode.PAUSE)
         {
            messager.submitMessage(topics.getSessionCurrentMode(), SessionMode.PAUSE);
            return;
         }

         if (sessionDT.get() == null || bufferRecordTickPeriod.get() == null)
            return;

         YoBufferPropertiesReadOnly bufferProperties = currentBufferProperties.getAndSet(null);

         if (bufferProperties == null)
            return;

         switch (currentPhase)
         {
            case 0:
               if (initialize(bufferProperties))
                  currentPhase++;
               return;
            case 1:
               if (recordNextFrame(bufferProperties))
                  currentPhase++;
               return;
            case 2:
               isDoneTakingJavaFXSnapshots = true;
               if (isDoneExporting)
                  currentPhase++;
               return;
            default:
               stop();
         }
      }

      private boolean initialize(YoBufferPropertiesReadOnly bufferProperties)
      {
         if (inPoint < 0)
            inPoint = bufferProperties.getInPoint();
         if (outPoint < 0)
            outPoint = bufferProperties.getOutPoint();

         double dt = Conversions.nanosecondsToSeconds(sessionDT.get()) * bufferRecordTickPeriod.get();
         double recordDT = request.getRealTimeRate() / request.getFrameRate();
         bufferIndexIncrement = (int) Math.ceil(recordDT / dt);

         currentRecordingBufferIndex = inPoint;
         numberOfBufferTicks = SharedMemoryTools.computeSubLength(inPoint, outPoint, bufferProperties.getSize());
         messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), currentRecordingBufferIndex);

         return true;
      }

      private boolean recordNextFrame(YoBufferPropertiesReadOnly bufferProperties)
      {
         if (currentRecordingBufferIndex != bufferProperties.getCurrentIndex())
            return false;

         jfxImageQueue.add(mainView3DRoot.snapshot(params, new WritableImage(request.getWidth(), request.getHeight())));

         numberOfBufferTicks -= bufferIndexIncrement;

         if (numberOfBufferTicks < 0)
            return true;

         currentRecordingBufferIndex = SharedMemoryTools.increment(currentRecordingBufferIndex, bufferIndexIncrement, bufferProperties.getSize());
         messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), currentRecordingBufferIndex);

         return false;
      }

      @Override
      public void stop()
      {
         super.stop();
         stopListener.run();
      }
   }
}
