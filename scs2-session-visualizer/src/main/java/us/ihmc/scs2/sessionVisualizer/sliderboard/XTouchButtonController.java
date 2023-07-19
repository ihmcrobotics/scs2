package us.ihmc.scs2.sessionVisualizer.sliderboard;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

import us.ihmc.scs2.sessionVisualizer.sliderboard.XTouchCompactSliderboardController.XTouchButton;

public class XTouchButtonController extends BehringerChannelController
{
   private final MidiChannelConfig button;
   
   
   private boolean changeCurrentDeviceState = false;
   private boolean currentDeviceValue = false;

   public XTouchButtonController(XTouchButton button, Receiver midiOut)
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

      if (channel != message.getData1()) 
         return false;

      changeCurrentDeviceState = true;
      return true;
   }

   @Override
   public void update()
   {
      if (changeCurrentDeviceState)
      {
         getControlVariable().setValue(getControlVariable().getValue() == 0 ? 1 : 0);
         changeCurrentDeviceState = false;
      }
      
      if ((getControlVariable().getValue() != 0) != currentDeviceValue)
      {
         pushControlVariableToDevice();
      }
   }
   
   @Override
   protected void pushValueToDevice(int value)
   {
      if (!enable || value == -1)
         return;

      try
      {
         ShortMessage message = new ShortMessage();
         boolean state = value != 0;
         
         if(state)
         {
            message.setMessage(ShortMessage.NOTE_ON, 0, channel, 127);
         }
         else
         {
            message.setMessage(ShortMessage.NOTE_OFF, 0, channel, 0);
         }
         
         midiOut.send(message, -1);
         
         
         currentDeviceValue = value != 0; 
      }
      catch (InvalidMidiDataException e)
      {
         e.printStackTrace();
      }
   }

}
