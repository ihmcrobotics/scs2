package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;

public abstract class BehringerChannelController
{
   protected final SliderboardVariable controlVariable;
   protected final Receiver midiOut;
   protected final int channel;

   protected boolean enable;

   protected int currentDeviceValue = -1;
   protected int newDeviceValue = -1;

   public BehringerChannelController(SliderboardVariable controlVariable, int channel, Receiver midiOut)
   {
      this.controlVariable = controlVariable;
      this.channel = channel;
      this.midiOut = midiOut;
   }

   public boolean handleMessage(ShortMessage message, long timestamp)
   {
      if (!enable)
         return false;

      if (channel != message.getData1()) // Should it use getChannel instead
         return false;

      newDeviceValue = MathTools.clamp(message.getData2(), controlVariable.getMin(), controlVariable.getMax());
      return true;
   }

   public void enable()
   {
      if (enable)
         return;

      enable = true;
      pushControlVariableToDevice();
      newDeviceValue = -1;
   }

   public void disable()
   {
      if (!enable)
         return;

      enable = false;
      newDeviceValue = -1;
   }

   public void update()
   {
      if (newDeviceValue != -1)
      {
         currentDeviceValue = newDeviceValue;
         controlVariable.setValue(currentDeviceValue);
         newDeviceValue = -1;
      }
      else if (controlVariable.getValue() != currentDeviceValue)
      {
         pushControlVariableToDevice();
      }
   }

   protected void pushControlVariableToDevice()
   {
      pushValueToDevice(controlVariable.getValue());
   }

   protected void pushValueToDevice(int value)
   {
      if (!enable || value == -1)
         return;

      currentDeviceValue = value; // TODO Not sure if that's the best way
      try
      {
         ShortMessage message = new ShortMessage();
         int msgData;
         if (controlVariable.getMax() - controlVariable.getMin() == 127)
         {
            msgData = value - controlVariable.getMin();
         }
         else if (controlVariable.getMax() - controlVariable.getMin() == 1)
         {
            msgData = value - controlVariable.getMin() == 0 ? 0 : 127;
         }
         else
         {
            int alpha = (value - controlVariable.getMin()) / (controlVariable.getMax() - controlVariable.getMax());
            msgData = (int) Math.round(EuclidCoreTools.interpolate(0, 127, alpha));
         }
         message.setMessage(176, 0, channel, msgData);
         midiOut.send(message, -1);
      }
      catch (InvalidMidiDataException e)
      {
         e.printStackTrace();
      }
   }

   public SliderboardVariable getControlVariable()
   {
      return controlVariable;
   }
}
