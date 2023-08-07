package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Receiver;
import javax.sound.midi.Transmitter;

import us.ihmc.log.LogTools;

public class BehringerMidiHelpers
{

   public static boolean DEBUG = false;

   public static Transmitter connectToMidiInDevice(MidiDevice.Info info)
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

   public static Receiver connectToMidiOutDevice(MidiDevice.Info info)
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

   public static boolean openDevice(MidiDevice midiDevice)
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

   public static MidiDevice getDevice(MidiDevice.Info info)
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
