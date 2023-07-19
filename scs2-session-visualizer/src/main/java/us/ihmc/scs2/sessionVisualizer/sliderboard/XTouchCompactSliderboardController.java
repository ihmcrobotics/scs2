package us.ihmc.scs2.sessionVisualizer.sliderboard;

import java.lang.reflect.Array;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Transmitter;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardType;

public class XTouchCompactSliderboardController
{

   public enum XTouchKnob implements MidiChannelConfig
   {
      KNOB_1, KNOB_2, KNOB_3, KNOB_4, KNOB_5, KNOB_6, KNOB_7, KNOB_8, KNOB_9, KNOB_10, KNOB_11, KNOB_12, KNOB_13, KNOB_14, KNOB_15, KNOB_16;

      public static final XTouchKnob[] values = values();
      private static final int CHANNEL_OFFSET = 9;

      private final int channel;

      private XTouchKnob()
      {
         int knob = ordinal() + 1;
         this.channel = knob + CHANNEL_OFFSET;
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

      public static XTouchKnob fromCommand(int command, int data1)
      {
         if(command != ShortMessage.CONTROL_CHANGE)
         {
            return null;
         }
         
         int knob = data1 - CHANNEL_OFFSET;
         int ordinal = knob - 1;

         if (ordinal < 0 || ordinal >= values.length)
            return null;

         return values[ordinal];
      }
   };

   public enum XTouchSlider implements MidiChannelConfig
   {
      SLIDER_1, SLIDER_2, SLIDER_3, SLIDER_4, SLIDER_5, SLIDER_6, SLIDER_7, SLIDER_8, SLIDER_MAIN;

      public static final XTouchSlider[] values = values();
      private static final int CHANNEL_OFFSET = 0;

      private final int channel;

      private XTouchSlider()
      {
         int slider = ordinal() + 1;

         this.channel = slider + CHANNEL_OFFSET;
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

      public static XTouchSlider fromCommand(int channel, int data1)
      {
         if(channel != ShortMessage.CONTROL_CHANGE)
         {
            return null;
         }
         
         int slider = data1 - CHANNEL_OFFSET;
         int ordinal = slider - 1;

         if (ordinal < 0 || ordinal >= values.length)
            return null;

         return values[ordinal];
      }
   };

   public enum XTouchButton implements MidiChannelConfig
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
      BUTTON_16,
      // Third row
      BUTTON_17,
      BUTTON_18,
      BUTTON_19,
      BUTTON_20,
      BUTTON_21,
      BUTTON_22,
      BUTTON_23,
      BUTTON_24,
      // Fourth row
      BUTTON_25,
      BUTTON_26,
      BUTTON_27,
      BUTTON_28,
      BUTTON_29,
      BUTTON_30,
      BUTTON_31,
      BUTTON_32,
      // Fifth row
      BUTTON_33,
      BUTTON_34,
      BUTTON_35,
      BUTTON_36,
      BUTTON_37,
      BUTTON_38,
      BUTTON_39;

      public static final XTouchButton[] values = values();
      private static final int CHANNEL_OFFSET = 15;

      private final int channel;

      private XTouchButton()
      {
         int button = ordinal() + 1;
         channel = button + CHANNEL_OFFSET;
      }

      public int getChannel()
      {
         return channel;
      }

      public static XTouchButton fromCommand(int channel, int data1)
      {
         if(channel != ShortMessage.NOTE_OFF)
         {
            return null;
         }
         
         int button = data1 - CHANNEL_OFFSET;
         int ordinal = button - 1;

         if (ordinal < 0 || ordinal >= values.length)
            return null;

         return values[ordinal];
      }
   }

   private final Receiver midiOut;
   private final Transmitter midiIn;

   private final ScheduledExecutorService executor = Executors.newScheduledThreadPool(1,
                                                                                      ThreadTools.createNamedDaemonThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> currentTask;

   private final BehringerSliderController[] sliderControllers = new BehringerSliderController[9];
   private final XTouchButtonController[] buttonControllers = new XTouchButtonController[39];
   private final BehringerKnobController[] knobControllers = new BehringerKnobController[16];
   private final BehringerChannelController[] allControllers;

   private final Receiver receiver = new Receiver()
   {
      @Override
      public void send(MidiMessage message, long timeStamp)
      {
         if (!(message instanceof ShortMessage))
            return;

         ShortMessage shortMessage = (ShortMessage) message;

         XTouchSlider slider = XTouchSlider.fromCommand(shortMessage.getCommand(), shortMessage.getData1());

         if (slider != null)
         {
            if (sliderControllers[slider.ordinal()].handleMessage(shortMessage, timeStamp))
               return;
         }

         XTouchButton button = XTouchButton.fromCommand(shortMessage.getCommand(), shortMessage.getData1());

         if (button != null)
         {
            if (buttonControllers[button.ordinal()].handleMessage(shortMessage, timeStamp))
               return;
         }

         XTouchKnob knob = XTouchKnob.fromCommand(shortMessage.getCommand(), shortMessage.getData1());

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

   private XTouchCompactSliderboardController(Receiver midiOut, Transmitter midiIn)
   {
      this.midiOut = midiOut;
      this.midiIn = midiIn;

      midiIn.setReceiver(receiver);

      for (XTouchSlider slider : XTouchSlider.values)
      {
         sliderControllers[slider.ordinal()] = new BehringerSliderController(slider, midiOut);
      }

      for (XTouchButton button : XTouchButton.values)
      {
         buttonControllers[button.ordinal()] = new XTouchButtonController(button, midiOut);
      }

      for (XTouchKnob knob : XTouchKnob.values)
      {
         knobControllers[knob.ordinal()] = new BehringerKnobController(knob, midiOut);
      }

      allControllers = combineArrays(BehringerChannelController.class, sliderControllers, buttonControllers, knobControllers);
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

   public SliderboardVariable getSlider(XTouchSlider slider)
   {
      return sliderControllers[slider.ordinal()].getControlVariable();
   }

   public SliderboardVariable getButton(XTouchButton button)
   {
      return buttonControllers[button.ordinal()].getControlVariable();
   }

   public SliderboardVariable getKnob(XTouchKnob knob)
   {
      return knobControllers[knob.ordinal()].getControlVariable();
   }

   public void update()
   {
      for (BehringerChannelController controller : allControllers)
      {
         controller.update();
      }
   }

   public void start()
   {
      if (currentTask != null)
         return;

      for (BehringerChannelController controller : allControllers)
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

      for (BehringerChannelController controller : allControllers)
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

   public static boolean isXTouchCompactSliderboard(MidiDevice.Info info)
   {
      String name = info.getName();
      String description = info.getDescription();

      return name.contains(YoSliderboardType.XTOUCHCOMPACT.getTypeString()) || description.contains(YoSliderboardType.XTOUCHCOMPACT.getTypeString());
   }

   public static void closeMidiDevices()
   {
      MidiDevice.Info[] infos = MidiSystem.getMidiDeviceInfo();

      for (MidiDevice.Info info : infos)
      {
         if (!isXTouchCompactSliderboard(info))
            continue;

         MidiDevice midiDevice = BehringerMidiHelpers.getDevice(info);

         if (midiDevice != null && midiDevice.isOpen())
         {
            midiDevice.close();
            continue;
         }
      }
   }

   public static XTouchCompactSliderboardController searchAndConnectToDevice()
   {
      MidiDevice.Info[] infos = MidiSystem.getMidiDeviceInfo();

      Receiver out = null;
      Transmitter in = null;

      for (MidiDevice.Info info : infos)
      {
         if (out != null && in != null)
            break;

         if (!isXTouchCompactSliderboard(info))
            continue;

         if (out == null)
         {
            out = BehringerMidiHelpers.connectToMidiOutDevice(info);
            if (out != null)
               continue;
         }

         if (in == null)
            in = BehringerMidiHelpers.connectToMidiInDevice(info);
      }

      if (out != null && in != null)
         return new XTouchCompactSliderboardController(out, in);
      else
         return null;
   }
}
