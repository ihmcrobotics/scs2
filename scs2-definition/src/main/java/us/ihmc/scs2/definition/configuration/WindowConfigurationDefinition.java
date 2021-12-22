package us.ihmc.scs2.definition.configuration;

public class WindowConfigurationDefinition
{
   private double positionX, positionY;
   private double width, height;
   private boolean maximized;

   public void setPositionX(double positionX)
   {
      this.positionX = positionX;
   }

   public void setPositionY(double positionY)
   {
      this.positionY = positionY;
   }

   public void setWidth(double width)
   {
      this.width = width;
   }

   public void setHeight(double height)
   {
      this.height = height;
   }

   public void setMaximized(boolean maximized)
   {
      this.maximized = maximized;
   }

   public double getPositionX()
   {
      return positionX;
   }

   public double getPositionY()
   {
      return positionY;
   }

   public double getWidth()
   {
      return width;
   }

   public double getHeight()
   {
      return height;
   }

   public boolean isMaximized()
   {
      return maximized;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof WindowConfigurationDefinition)
      {
         WindowConfigurationDefinition other = (WindowConfigurationDefinition) object;
         if (positionX != other.positionX)
            return false;
         if (positionY != other.positionY)
            return false;
         if (width != other.width)
            return false;
         if (height != other.height)
            return false;
         if (maximized != other.maximized)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "positionX: " + positionX + ", positionY: " + positionY + ", width: " + width + ", height: " + height + ", maximized: " + maximized;
   }
}
