package us.ihmc.scs2.definition.configuration;

public class WindowConfigurationDefinition
{
   private double positionX, positionY;
   private double width, height;
   private boolean maximized;
   /** This is informative and is not used by some utility classes. */
   private boolean isShowing;

   public WindowConfigurationDefinition()
   {
   }

   public WindowConfigurationDefinition(WindowConfigurationDefinition other)
   {
      set(other);
   }

   public void set(WindowConfigurationDefinition other)
   {
      positionX = other.positionX;
      positionY = other.positionY;
      width = other.width;
      height = other.height;
      maximized = other.maximized;
      isShowing = other.isShowing;
   }

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

   public void setShowing(boolean isShowing)
   {
      this.isShowing = isShowing;
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

   public boolean isShowing()
   {
      return isShowing;
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
         if (isShowing != other.isShowing)
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
      return "positionX: " + positionX + ", positionY: " + positionY + ", width: " + width + ", height: " + height + ", maximized: " + maximized
             + ", isShowing: " + isShowing;
   }
}
