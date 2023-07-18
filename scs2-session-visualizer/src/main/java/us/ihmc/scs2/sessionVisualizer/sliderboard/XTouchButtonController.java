package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

import us.ihmc.commons.MathTools;

public class XTouchButtonController extends BehringerChannelController
{
   private final MidiChannelConfig button;
   
   
   private boolean changeCurrentDeviceState = false;

   public XTouchButtonController(MidiChannelConfig button, Receiver midiOut)
   {
      super(new SliderboardVariable(0, 1), button.getChannel(), midiOut);

      this.button = button;
   }

   public MidiChannelConfig getButton()
   {
      return button;
   }

   @Override
   public boolean handleMessage(ShortMessage message, long timestamp)
   {
      if (!enable)
         return false;

      if (channel != message.getData1()) // Should it use getChannel instead
         return false;

      changeCurrentDeviceState = true;
      return true;
   }

   public void update()
   {
      if (changeCurrentDeviceState)
      {
         currentDeviceValue = currentDeviceValue == 0 ? 1 : 0;
         controlVariable.setValue(currentDeviceValue);
         
         changeCurrentDeviceState = false;
      }
      else if (controlVariable.getValue() != currentDeviceValue)
      {
         pushControlVariableToDevice();
      }
   }
}
