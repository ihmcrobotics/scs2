package us.ihmc.scs2.definition.yoComposite;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoTuple2D")
public class YoTuple2DDefinition extends YoCompositeDefinition
{
   public static final String YoTuple2D = "YoTuple2D";
   public static final String[] YoTuple2DIdentifiers = new String[] {"x", "y"};

   private String x, y;
   private String referenceFrame;

   public YoTuple2DDefinition()
   {
   }

   public YoTuple2DDefinition(String x, String y)
   {
      this(x, y, null);
   }

   public YoTuple2DDefinition(String x, String y, String referenceFrame)
   {
      this.x = x;
      this.y = y;
      this.referenceFrame = referenceFrame;
   }

   public void setX(double x)
   {
      this.x = Double.toString(x);
   }

   @XmlElement
   public void setX(String x)
   {
      this.x = x;
   }

   public void setY(double y)
   {
      this.y = Double.toString(y);
   }

   @XmlElement
   public void setY(String y)
   {
      this.y = y;
   }

   @XmlElement
   public void setReferenceFrame(String referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public String getX()
   {
      return x;
   }

   public String getY()
   {
      return y;
   }

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoTuple2D;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoTuple2DIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {x, y};
   }

   public static YoTuple2DDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoTuple2D))
      {
         value = value.substring(value.indexOf("=") +1).trim();
         String x = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") +1).trim();
         String y = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") +1).trim();
         String frame = value.substring(0, value.length() - 1);

         if (x.toLowerCase().equals("null"))
            x = null;
         if (y.toLowerCase().equals("null"))
            y = null;
         if (frame.toLowerCase().equals("null"))
            frame = null;

         return new YoTuple2DDefinition(x, y, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown tuple 2D format: " + value);
      }
   }
}
