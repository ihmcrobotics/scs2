package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;

public class SDFSensor implements SDFItem
{
   private String name;
   private String type;
   private String updateRate;
   private String pose;
   private List<SDFCamera> camera;
   private SDFRay ray;
   private SDFIMU imu;

   public String getName()
   {
      return name;
   }

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   public String getType()
   {
      return type;
   }

   @XmlAttribute(name = "type")
   public void setType(String type)
   {
      this.type = type;
   }

   public String getUpdateRate()
   {
      return updateRate;
   }

   @XmlElement(name = "update_rate")
   public void setUpdateRate(String updateRate)
   {
      this.updateRate = updateRate;
   }

   public String getPose()
   {
      return pose;
   }

   @XmlElement(name = "pose")
   public void setPose(String pose)
   {
      this.pose = pose;
   }

   public List<SDFCamera> getCamera()
   {
      return camera;
   }

   @XmlElement(name = "camera")
   public void setCamera(List<SDFCamera> camera)
   {
      this.camera = camera;
   }

   @XmlElement(name = "ray")
   public void setRay(SDFRay ray)
   {
      this.ray = ray;
   }

   public SDFRay getRay()
   {
      return ray;
   }

   public SDFIMU getImu()
   {
      return imu;
   }

   @XmlElement(name = "imu")
   public void setImu(SDFIMU imu)
   {
      this.imu = imu;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, type: %s, updateRate: %s, pose: %s, camera: %s, ray: %s, imu: %s]", name, type, updateRate, pose, camera, ray, imu);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return SDFItem.combineItemListsURIHolders(camera, Arrays.asList(ray, imu));
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   public static class SDFRay implements SDFItem
   {
      private String pose;
      private SDFRange range;
      private SDFScan scan;
      private SDFNoise noise;

      @XmlElement(name = "pose")
      public void setPose(String pose)
      {
         this.pose = pose;
      }

      @XmlElement(name = "range")
      public void setRange(SDFRange range)
      {
         this.range = range;
      }

      @XmlElement(name = "scan")
      public void setScan(SDFScan scan)
      {
         this.scan = scan;
      }

      public String getPose()
      {
         return pose;
      }

      public SDFRange getRange()
      {
         return range;
      }

      public SDFScan getScan()
      {
         return scan;
      }

      public SDFNoise getNoise()
      {
         return noise;
      }

      @XmlElement(name = "noise")
      public void setNoise(SDFNoise noise)
      {
         this.noise = noise;
      }

      @Override
      public String getContentAsString()
      {
         return format("[pose: %s, range: %s, scan: %s, noise: %s]", pose, range, scan, noise);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return SDFItem.combineItemURIHolders(range, scan, noise);
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class SDFRange implements SDFItem
      {
         private String min;
         private String max;
         private String resolution;

         @XmlElement(name = "min")
         public void setMin(String min)
         {
            this.min = min;
         }

         @XmlElement(name = "max")
         public void setMax(String max)
         {
            this.max = max;
         }

         @XmlElement(name = "resolution")
         public void setResolution(String resolution)
         {
            this.resolution = resolution;
         }

         public String getMin()
         {
            return min;
         }

         public String getMax()
         {
            return max;
         }

         public String getResolution()
         {
            return resolution;
         }

         @Override
         public String getContentAsString()
         {
            return format("[min: %s, max: %s, resolution: %s]", min, max, resolution);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }

      public static class SDFScan implements SDFItem
      {
         private SDFHorizontalScan horizontal;
         private SDFVerticalScan vertical;

         @XmlElement(name = "horizontal")
         public void setHorizontal(SDFHorizontalScan horizontal)
         {
            this.horizontal = horizontal;
         }

         public SDFHorizontalScan getHorizontal()
         {
            return horizontal;
         }

         @XmlElement(name = "vertical")
         public SDFVerticalScan getVertical()
         {
            return vertical;
         }

         public void setVertical(SDFVerticalScan vertical)
         {
            this.vertical = vertical;
         }

         @Override
         public String getContentAsString()
         {
            return format("[horizontal: %s, vertical: %s]", horizontal, vertical);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return SDFItem.combineItemURIHolders(horizontal, vertical);
         }

         @Override
         public String toString()
         {
            return itemToString();
         }

         public static class SDFHorizontalScan implements SDFItem
         {
            private String samples;
            private String resolution;
            private String minAngle;
            private String maxAngle;

            @XmlElement(name = "samples")
            public void setSamples(String samples)
            {
               this.samples = samples;
            }

            @XmlElement(name = "resolution")
            public void setResolution(String resolution)
            {
               this.resolution = resolution;
            }

            @XmlElement(name = "min_angle")
            public void setMinAngle(String minAngle)
            {
               this.minAngle = minAngle;
            }

            @XmlElement(name = "max_angle")
            public void setMaxAngle(String maxAngle)
            {
               this.maxAngle = maxAngle;
            }

            public String getSamples()
            {
               return samples;
            }

            public String getResolution()
            {
               return resolution;
            }

            public String getMinAngle()
            {
               return minAngle;
            }

            public String getMaxAngle()
            {
               return maxAngle;
            }

            @Override
            public String getContentAsString()
            {
               return format("[samples: %s, resolution: %s, minAngle: %s, maxAngle: %s]", samples, resolution, minAngle, maxAngle);
            }

            @Override
            public List<SDFURIHolder> getURIHolders()
            {
               return Collections.emptyList();
            }

            @Override
            public String toString()
            {
               return itemToString();
            }
         }

         public static class SDFVerticalScan implements SDFItem
         {
            private String samples;
            private String resolution;
            private String minAngle;
            private String maxAngle;

            @XmlElement(name = "samples")
            public void setSamples(String samples)
            {
               this.samples = samples;
            }

            @XmlElement(name = "resolution")
            public void setResolution(String resolution)
            {
               this.resolution = resolution;
            }

            @XmlElement(name = "min_angle")
            public void setMinAngle(String minAngle)
            {
               this.minAngle = minAngle;
            }

            @XmlElement(name = "max_angle")
            public void setMaxAngle(String maxAngle)
            {
               this.maxAngle = maxAngle;
            }

            public String getSamples()
            {
               return samples;
            }

            public String getResolution()
            {
               return resolution;
            }

            public String getMinAngle()
            {
               return minAngle;
            }

            public String getMaxAngle()
            {
               return maxAngle;
            }

            @Override
            public String getContentAsString()
            {
               return format("[samples: %s, resolution: %s, minAngle: %s, maxAngle: %s]", samples, resolution, minAngle, maxAngle);
            }

            @Override
            public List<SDFURIHolder> getURIHolders()
            {
               return Collections.emptyList();
            }

            @Override
            public String toString()
            {
               return itemToString();
            }
         }
      }

      public static class SDFNoise implements SDFItem
      {
         private String type;
         private String mean;
         private String stddev;

         @XmlElement(name = "type")
         public void setType(String type)
         {
            this.type = type;
         }

         @XmlElement(name = "mean")
         public void setMean(String mean)
         {
            this.mean = mean;
         }

         @XmlElement(name = "stddev")
         public void setStddev(String stddev)
         {
            this.stddev = stddev;
         }

         public String getType()
         {
            return type;
         }

         public String getMean()
         {
            return mean;
         }

         public String getStddev()
         {
            return stddev;
         }

         @Override
         public String getContentAsString()
         {
            return format("[type: %s, mean: %s, stddev]", type, mean, stddev);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }
   }

   public static class SDFCamera implements SDFItem
   {
      private String name;
      private String pose;
      private String horizontalFov;
      private SDFSensorImage image;
      private SDFClip clip;

      public String getPose()
      {
         return pose;
      }

      @XmlElement(name = "pose")
      public void setPose(String pose)
      {
         this.pose = pose;
      }

      public String getHorizontalFov()
      {
         return horizontalFov;
      }

      @XmlElement(name = "horizontal_fov")
      public void setHorizontalFov(String horizontalFov)
      {
         this.horizontalFov = horizontalFov;
      }

      public SDFSensorImage getImage()
      {
         return image;
      }

      @XmlElement(name = "image")
      public void setImage(SDFSensorImage image)
      {
         this.image = image;
      }

      public SDFClip getClip()
      {
         return clip;
      }

      @XmlElement(name = "clip")
      public void setClip(SDFClip clip)
      {
         this.clip = clip;
      }

      @XmlAttribute(name = "name")
      public String getName()
      {
         return name;
      }

      public void setName(String name)
      {
         this.name = name;
      }

      @Override
      public String getContentAsString()
      {
         return format("[name: %s, pose: %s, horizontalFov: %s, image: %s, clip: %s]", name, pose, horizontalFov, image, clip);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class SDFSensorImage implements SDFItem
      {
         private String width;
         private String height;
         private String format;

         public String getWidth()
         {
            return width;
         }

         @XmlElement(name = "width")
         public void setWidth(String width)
         {
            this.width = width;
         }

         public String getHeight()
         {
            return height;
         }

         @XmlElement(name = "height")
         public void setHeight(String height)
         {
            this.height = height;
         }

         public String getFormat()
         {
            return format;
         }

         @XmlElement(name = "format")
         public void setFormat(String format)
         {
            this.format = format;
         }

         @Override
         public String getContentAsString()
         {
            return format("[width: %s, height: %s, format: %s]", width, height, format);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }

      public static class SDFClip implements SDFItem
      {
         private String near;
         private String far;

         public String getNear()
         {
            return near;
         }

         @XmlElement(name = "near")
         public void setNear(String near)
         {
            this.near = near;
         }

         public String getFar()
         {
            return far;
         }

         @XmlElement(name = "far")
         public void setFar(String far)
         {
            this.far = far;
         }

         @Override
         public String getContentAsString()
         {
            return format("[near: %s, far: %s]", near, far);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }
   }

   public static class SDFIMU implements SDFItem
   {
      private SDFIMUNoise noise;

      public SDFIMUNoise getNoise()
      {
         return noise;
      }

      @XmlElement(name = "noise")
      public void setNoise(SDFIMUNoise noise)
      {
         this.noise = noise;
      }

      @Override
      public String getContentAsString()
      {
         return format("[noise: %s]", noise);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class SDFIMUNoise implements SDFItem
      {
         private String type;
         private SDFNoiseParameters rate;
         private SDFNoiseParameters accel;

         public String getType()
         {
            return type;
         }

         public SDFNoiseParameters getRate()
         {
            return rate;
         }

         public SDFNoiseParameters getAccel()
         {
            return accel;
         }

         @XmlElement(name = "type")
         public void setType(String type)
         {
            this.type = type;
         }

         @XmlElement(name = "rate")
         public void setRate(SDFNoiseParameters rate)
         {
            this.rate = rate;
         }

         @XmlElement(name = "accel")
         public void setAccel(SDFNoiseParameters accel)
         {
            this.accel = accel;
         }

         @Override
         public String getContentAsString()
         {
            return format("[type: %s, rate: %s, accel: %s]", type, rate, accel);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }

         public static class SDFNoiseParameters implements SDFItem
         {
            private String mean;
            private String stddev;
            private String bias_mean;
            private String bias_stddev;

            public String getMean()
            {
               return mean;
            }

            public String getStddev()
            {
               return stddev;
            }

            public String getBias_mean()
            {
               return bias_mean;
            }

            public String getBias_stddev()
            {
               return bias_stddev;
            }

            @XmlElement(name = "mean")
            public void setMean(String mean)
            {
               this.mean = mean;
            }

            @XmlElement(name = "stddev")
            public void setStddev(String stddev)
            {
               this.stddev = stddev;
            }

            @XmlElement(name = "bias_mean")
            public void setBias_mean(String bias_mean)
            {
               this.bias_mean = bias_mean;
            }

            @XmlElement(name = "bias_stddev")
            public void setBias_stddev(String bias_stddev)
            {
               this.bias_stddev = bias_stddev;
            }

            @Override
            public String getContentAsString()
            {
               return format("[mean: %s, stddev: %s, bias_mean: %s, bias_stddev: %s]", mean, stddev, bias_mean, bias_stddev);
            }

            @Override
            public List<SDFURIHolder> getURIHolders()
            {
               return Collections.emptyList();
            }

            @Override
            public String toString()
            {
               return itemToString();
            }
         }
      }
   }
}
