package us.ihmc.scs2.definition.yoComposite;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoYawPitchRoll")
public class YoYawPitchRollDefinition extends YoOrientation3DDefinition
{
   public static final String YoYawPitchRoll = "YoYawPitchRoll";
   public static final String[] YoYawPitchRollIdentifiers = new String[] {"yaw", "pitch", "roll"};

   private String yaw, pitch, roll;
   private String referenceFrame;

   public YoYawPitchRollDefinition()
   {
   }

   public YoYawPitchRollDefinition(String yaw, String pitch, String roll)
   {
      this(yaw, pitch, roll, null);
   }

   public YoYawPitchRollDefinition(String yaw, String pitch, String roll, String referenceFrame)
   {
      this.yaw = yaw;
      this.pitch = pitch;
      this.roll = roll;
      this.referenceFrame = referenceFrame;
   }

   public void setYaw(double yaw)
   {
      this.yaw = Double.toString(yaw);
   }

   @XmlElement
   public void setYaw(String yaw)
   {
      this.yaw = yaw;
   }

   public void setPitch(double pitch)
   {
      this.pitch = Double.toString(pitch);
   }

   @XmlElement
   public void setPitch(String pitch)
   {
      this.pitch = pitch;
   }

   public void setRoll(double roll)
   {
      this.roll = Double.toString(roll);
   }

   @XmlElement
   public void setRoll(String roll)
   {
      this.roll = roll;
   }

   @XmlElement
   public void setReferenceFrame(String referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public String getYaw()
   {
      return yaw;
   }

   public String getPitch()
   {
      return pitch;
   }

   public String getRoll()
   {
      return roll;
   }

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoYawPitchRoll;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoYawPitchRollIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {yaw, pitch, roll};
   }

   public static YoYawPitchRollDefinition parse(String value)
   {
      value = value.trim();

      if (value.startsWith(YoYawPitchRoll))
      {
         value = value.substring(value.indexOf("=") + 1).trim();
         String yaw = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String pitch = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String roll = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String frame = value.substring(0, value.length() - 1);

         if (yaw.toLowerCase().equals("null"))
            yaw = null;
         if (pitch.toLowerCase().equals("null"))
            pitch = null;
         if (roll.toLowerCase().equals("null"))
            roll = null;
         if (frame.toLowerCase().equals("null"))
            frame = null;

         return new YoYawPitchRollDefinition(yaw, pitch, roll, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown yaw-pitch-roll format: " + value);
      }
   }
}
