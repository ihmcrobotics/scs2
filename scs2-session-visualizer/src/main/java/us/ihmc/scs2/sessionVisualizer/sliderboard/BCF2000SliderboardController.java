package us.ihmc.scs2.sessionVisualizer.sliderboard;

import java.lang.reflect.Array;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Transmitter;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

public class BCF2000SliderboardController
{
   public static final String BCF2000 = "BCF2000";

   public static boolean DEBUG = true;

   public enum Knob
   {
      KNOB_1, KNOB_2, KNOB_3, KNOB_4, KNOB_5, KNOB_6, KNOB_7, KNOB_8;

      public static final Knob[] values = values();
      private static final int CHANNEL_OFFSET = 1;

      private final int channel;

      private Knob()
      {
         this.channel = ordinal() + CHANNEL_OFFSET;
      }

      public int getChannel()
      {
         return channel;
      }

      public int getMin()
      {
         return 0;
      }

      public int getMax()
      {
         return 127;
      }

      public static Knob fromChannel(int channel)
      {
         if (channel < CHANNEL_OFFSET || channel > CHANNEL_OFFSET + 7)
            return null;
         return values[channel - CHANNEL_OFFSET];
      }
   };

   public enum Slider
   {
      SLIDER_1, SLIDER_2, SLIDER_3, SLIDER_4, SLIDER_5, SLIDER_6, SLIDER_7, SLIDER_8;

      public static final Slider[] values = values();
      private static final int CHANNEL_OFFSET = 81;

      private final int channel;

      private Slider()
      {
         this.channel = ordinal() + CHANNEL_OFFSET;
      }

      public int getChannel()
      {
         return channel;
      }

      public int getMin()
      {
         return 0;
      }

      public int getMax()
      {
         return 127;
      }

      public static Slider fromChannel(int channel)
      {
         if (channel < CHANNEL_OFFSET || channel > CHANNEL_OFFSET + 7)
            return null;
         return values[channel - CHANNEL_OFFSET];
      }
   };

   public enum Button
   {
      // First row
      BUTTON_1,
      BUTTON_2,
      BUTTON_3,
      BUTTON_4,
      BUTTON_5,
      BUTTON_6,
      BUTTON_7,
      BUTTON_8,
      // Second row
      BUTTON_9,
      BUTTON_10,
      BUTTON_11,
      BUTTON_12,
      BUTTON_13,
      BUTTON_14,
      BUTTON_15,
      BUTTON_16;

      public static final Button[] values = values();
      private static final int CHANNEL_OFFSET = 65;

      private final int channel;

      private Button()
      {
         channel = ordinal() + CHANNEL_OFFSET;
      }

      public int getChannel()
      {
         return channel;
      }

      public static Button fromChannel(int channel)
      {
         if (channel < CHANNEL_OFFSET || channel > CHANNEL_OFFSET + 15)
            return null;
         return values[channel - CHANNEL_OFFSET];
      }
   }

   public enum KnobButton
   {
      BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4, BUTTON_5, BUTTON_6, BUTTON_7, BUTTON_8;

      public static final KnobButton[] values = values();
      private static final int CHANNEL_OFFSET = 32;

      private final int channel;

      private KnobButton()
      {
         channel = ordinal() + CHANNEL_OFFSET;
      }

      public int getChannel()
      {
         return channel;
      }

      public static KnobButton fromChannel(int channel)
      {
         if (channel < CHANNEL_OFFSET || channel > CHANNEL_OFFSET + 7)
            return null;
         return values[channel - CHANNEL_OFFSET];
      }
   }

   private final Receiver midiOut;
   private final Transmitter midiIn;

   private final ScheduledExecutorService executor = Executors.newScheduledThreadPool(1,
                                                                                      ThreadTools.createNamedDaemonThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> currentTask;

   private final BCF2000SliderController[] sliderControllers = new BCF2000SliderController[8];
   private final BCF2000ButtonController[] buttonControllers = new BCF2000ButtonController[16];
   private final BCF2000KnobController[] knobControllers = new BCF2000KnobController[8];
   private final BCF2000ChannelController[] allControllers;

   private final Receiver receiver = new Receiver()
   {
      @Override
      public void send(MidiMessage message, long timeStamp)
      {
         if (!(message instanceof ShortMessage))
            return;

         ShortMessage shortMessage = (ShortMessage) message;

         Slider slider = Slider.fromChannel(shortMessage.getData1());

         if (slider != null)
         {
            if (sliderControllers[slider.ordinal()].handleMessage(shortMessage, timeStamp))
               return;
         }

         Button button = Button.fromChannel(shortMessage.getData1());

         if (button != null)
         {
            if (buttonControllers[button.ordinal()].handleMessage(shortMessage, timeStamp))
               return;
         }

         Knob knob = Knob.fromChannel(shortMessage.getData1());

         if (knob != null)
         {
            if (knobControllers[knob.ordinal()].handleMessage(shortMessage, timeStamp))
               return;
         }
      }

      @Override
      public void close()
      {
      }
   };

   private BCF2000SliderboardController(Receiver midiOut, Transmitter midiIn)
   {
      this.midiOut = midiOut;
      this.midiIn = midiIn;

      midiIn.setReceiver(receiver);

      for (Slider slider : Slider.values)
      {
         sliderControllers[slider.ordinal()] = new BCF2000SliderController(slider, midiOut);
      }

      for (Button button : Button.values)
      {
         buttonControllers[button.ordinal()] = new BCF2000ButtonController(button, midiOut);
      }

      for (Knob knob : Knob.values)
      {
         knobControllers[knob.ordinal()] = new BCF2000KnobController(knob, midiOut);
      }

      allControllers = combineArrays(BCF2000ChannelController.class, sliderControllers, buttonControllers, knobControllers);
   }

   @SuppressWarnings("unchecked")
   private static <T> T[] combineArrays(Class<T> componentType, T[]... inputArrays)
   {
      int outputLength = 0;
      for (T[] inputArray : inputArrays)
      {
         outputLength += inputArray.length;
      }

      T[] outputArray = (T[]) Array.newInstance(componentType, outputLength);

      int outputIndex = 0;
      for (T[] inputArray : inputArrays)
      {
         for (int i = 0; i < inputArray.length; i++)
         {
            outputArray[outputIndex++] = inputArray[i];
         }
      }

      return outputArray;
   }

   public SliderboardVariable getSlider(Slider slider)
   {
      return sliderControllers[slider.ordinal()].getControlVariable();
   }

   public SliderboardVariable getButton(Button button)
   {
      return buttonControllers[button.ordinal()].getControlVariable();
   }

   public SliderboardVariable getKnob(Knob knob)
   {
      return knobControllers[knob.ordinal()].getControlVariable();
   }

   public void update()
   {
      for (BCF2000ChannelController controller : allControllers)
      {
         controller.update();
      }
   }

   public void start()
   {
      if (currentTask != null)
         return;

      for (BCF2000ChannelController controller : allControllers)
      {
         controller.enable();
      }

      currentTask = executor.scheduleAtFixedRate(this::update, 0, 20, TimeUnit.MILLISECONDS);
   }

   public void stop()
   {
      if (currentTask != null)
      {
         currentTask.cancel(false);
         currentTask = null;
      }

      for (BCF2000ChannelController controller : allControllers)
      {
         controller.disable();
      }
   }

   public void closeAndDispose()
   {
      stop();
      executor.shutdown();

      if (midiOut != null)
         midiOut.close();
      if (midiIn != null)
         midiIn.close();
   }

   public static boolean isBCF2000Sliderboard(MidiDevice.Info info)
   {
      String name = info.getName();
      String description = info.getDescription();
      return name.contains(BCF2000) || description.contains(BCF2000);
   }

   public static BCF2000SliderboardController searchAndConnectToDevice()
   {
      MidiDevice.Info[] infos = MidiSystem.getMidiDeviceInfo();

      Receiver out = null;
      Transmitter in = null;

      for (MidiDevice.Info info : infos)
      {
         if (out != null && in != null)
            break;

         if (!isBCF2000Sliderboard(info))
            continue;

         if (out == null)
         {
            out = connectToMidiOutDevice(info);
            if (out != null)
               continue;
         }

         if (in == null)
            in = connectToMidiInDevice(info);
      }

      if (out != null && in != null)
         return new BCF2000SliderboardController(out, in);
      else
         return null;
   }

   private static Transmitter connectToMidiInDevice(MidiDevice.Info info)
   {
      MidiDevice midiDevice = getDevice(info);

      if (midiDevice == null)
         return null;

      if (midiDevice.getMaxTransmitters() == 0)
      {
         if (DEBUG)
         {
            LogTools.error("Cannot add transmitter to the device: " + midiDevice);
            return null;
         }
      }

      if (!openDevice(midiDevice))
         return null;

      if (DEBUG)
         LogTools.info("Device is Now open trying to obtain the transmitter.");

      Transmitter midiIn = null;

      try
      {
         midiIn = midiDevice.getTransmitter();
      }
      catch (MidiUnavailableException e)
      {
         midiDevice = null;
         midiIn = null;

         if (DEBUG)
         {
            LogTools.error("Error getting the device's transmitter.");
            e.printStackTrace();
         }

         return null;
      }

      if (DEBUG)
         LogTools.info("Obtained a handle to the device transmitter: " + info.getName() + ", description: " + info.getDescription() + ", class name: "
               + midiDevice.getClass().getSimpleName());

      return midiIn;
   }

   private static Receiver connectToMidiOutDevice(MidiDevice.Info info)
   {
      MidiDevice midiDevice = getDevice(info);

      if (midiDevice == null)
         return null;

      if (midiDevice.getMaxReceivers() == 0)
      {
         if (DEBUG)
         {
            LogTools.error("Cannot add receiver to the device: " + midiDevice);
            return null;
         }
      }

      if (!openDevice(midiDevice))
         return null;

      if (DEBUG)
         LogTools.info("Device is Now open trying to obtain the receiver.");

      Receiver midiOut = null;

      try
      {
         midiOut = midiDevice.getReceiver();
      }
      catch (MidiUnavailableException e)
      {
         midiDevice = null;
         midiOut = null;

         if (DEBUG)
         {
            LogTools.error("Error getting the device's receiver.");
            e.printStackTrace();
         }

         return null;
      }

      if (DEBUG)
         LogTools.info("Obtained a handle to the devices receiver: " + info.getName() + ", description: " + info.getDescription());

      return midiOut;
   }

   private static boolean openDevice(MidiDevice midiDevice)
   {
      if (midiDevice.isOpen())
         return true;

      if (DEBUG)
         LogTools.info("Opening Device: " + midiDevice);

      try
      {
         midiDevice.open();
         return true;
      }
      catch (MidiUnavailableException e)
      {
         midiDevice = null;

         if (DEBUG)
         {
            LogTools.error("Unable to open device: " + midiDevice);
            e.printStackTrace();
         }
         return false;
      }
   }

   private static MidiDevice getDevice(MidiDevice.Info info)
   {
      MidiDevice midiDevice;
      try
      {
         midiDevice = MidiSystem.getMidiDevice(info);
      }
      catch (MidiUnavailableException e)
      {
         if (DEBUG)
         {
            LogTools.error("Unable to get a handle to this Midi Device.");
            e.printStackTrace();
         }

         return null;
      }
      return midiDevice;
   }
}
