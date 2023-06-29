package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

/**
 * <a href="http://wiki.ros.org/urdf/XML/sensor"> ROS Specification sensor.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFSensor implements URDFItem
{
   public enum URDFSensorType
   {
      camera, multicamera, depth, imu, gpu_ray, ray, force_torque, contact;

      public static URDFSensorType parse(String value)
      {
         if (value == null)
            return null;
         for (URDFSensorType type : values())
         {
            if (type.name().equals(value))
               return type;
         }
         return null;
      }
   };

   private String name;
   private String type;
   private String pose;
   private String visualize;
   private String updateRate;
   private List<URDFCamera> camera;
   private URDFRay ray;
   private URDFIMU imu;

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement(name = "pose")
   public void setPose(String pose)
   {
      this.pose = pose;
   }

   @XmlAttribute(name = "type")
   public void setType(String type)
   {
      this.type = type;
   }

   public void setType(URDFSensorType type)
   {
      setType(type.name());
   }

   @XmlElement(name = "visualize")
   public void setVisualize(String visualize)
   {
      this.visualize = visualize;
   }

   @XmlElement(name = "update_rate")
   public void setUpdateRate(String updateRate)
   {
      this.updateRate = updateRate;
   }

   @XmlElement(name = "ray")
   public void setRay(URDFRay ray)
   {
      this.ray = ray;
   }

   @XmlElement(name = "camera")
   public void setCamera(List<URDFCamera> camera)
   {
      this.camera = camera;
   }

   @XmlElement(name = "imu")
   public void setImu(URDFIMU imu)
   {
      this.imu = imu;
   }

   public String getName()
   {
      return name;
   }

   public String getPose()
   {
      return pose;
   }

   public String getType()
   {
      return type;
   }

   public String getVisualize()
   {
      return visualize;
   }

   public String getUpdateRate()
   {
      return updateRate;
   }

   public URDFRay getRay()
   {
      return ray;
   }

   public List<URDFCamera> getCamera()
   {
      return camera;
   }

   public URDFIMU getImu()
   {
      return imu;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, type: %s, updateRate: %s, pose: %s, camera: %s, ray: %s, imu: %s]", name, type, updateRate, pose, camera, ray, imu);
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return URDFItem.combineItemListsFilenameHolders(camera, Arrays.asList(ray, imu));
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   public static class URDFRay implements URDFItem
   {
      private String pose;
      private URDFRange range;
      private URDFScan scan;
      private URDFNoise noise;

      @XmlElement(name = "pose")
      public void setPose(String pose)
      {
         this.pose = pose;
      }

      @XmlElement(name = "range")
      public void setRange(URDFRange range)
      {
         this.range = range;
      }

      @XmlElement(name = "scan")
      public void setScan(URDFScan scan)
      {
         this.scan = scan;
      }

      public String getPose()
      {
         return pose;
      }

      public URDFRange getRange()
      {
         return range;
      }

      public URDFScan getScan()
      {
         return scan;
      }

      public URDFNoise getNoise()
      {
         return noise;
      }

      @XmlElement(name = "noise")
      public void setNoise(URDFNoise noise)
      {
         this.noise = noise;
      }

      @Override
      public String getContentAsString()
      {
         return format("[pose: %s, range: %s, scan: %s, noise: %s]", pose, range, scan, noise);
      }

      @Override
      public List<URDFFilenameHolder> getFilenameHolders()
      {
         return URDFItem.combineItemFilenameHolders(range, scan, noise);
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class URDFRange implements URDFItem
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
         public List<URDFFilenameHolder> getFilenameHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }

      public static class URDFScan implements URDFItem
      {
         private URDFHorizontalScan horizontal;
         private URDFVerticalScan vertical;

         @XmlElement(name = "horizontal")
         public void setHorizontal(URDFHorizontalScan horizontal)
         {
            this.horizontal = horizontal;
         }

         @XmlElement(name = "vertical")
         public void setVertical(URDFVerticalScan vertical)
         {
            this.vertical = vertical;
         }

         public URDFHorizontalScan getHorizontal()
         {
            return horizontal;
         }

         public URDFVerticalScan getVertical()
         {
            return vertical;
         }

         @Override
         public String getContentAsString()
         {
            return format("[horizontal: %s, vertical: %s]", horizontal, vertical);
         }

         @Override
         public List<URDFFilenameHolder> getFilenameHolders()
         {
            return URDFItem.combineItemFilenameHolders(horizontal, vertical);
         }

         @Override
         public String toString()
         {
            return itemToString();
         }

         public static class URDFHorizontalScan implements URDFItem
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
            public List<URDFFilenameHolder> getFilenameHolders()
            {
               return Collections.emptyList();
            }

            @Override
            public String toString()
            {
               return itemToString();
            }
         }

         public static class URDFVerticalScan implements URDFItem
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
            public List<URDFFilenameHolder> getFilenameHolders()
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

      public static class URDFNoise implements URDFItem
      {
         public enum URDFNoiseType
         {
            gaussian;

            public static URDFNoiseType parse(String value)
            {
               if (value == null)
                  return null;
               for (URDFNoiseType type : values())
               {
                  if (type.name().equals(value))
                     return type;
               }
               return null;
            }
         };

         private String type;
         private String mean;
         private String stddev;

         @XmlElement(name = "type")
         public void setType(String type)
         {
            this.type = type;
         }

         public void setType(URDFNoiseType type)
         {
            setType(type.name());
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
         public List<URDFFilenameHolder> getFilenameHolders()
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

   public static class URDFCamera implements URDFItem
   {
      private String name;
      private String pose;
      private String horizontalFov;
      private URDFSensorImage image;
      private URDFClip clip;

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

      public URDFSensorImage getImage()
      {
         return image;
      }

      @XmlElement(name = "image")
      public void setImage(URDFSensorImage image)
      {
         this.image = image;
      }

      public URDFClip getClip()
      {
         return clip;
      }

      @XmlElement(name = "clip")
      public void setClip(URDFClip clip)
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
      public List<URDFFilenameHolder> getFilenameHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class URDFSensorImage implements URDFItem
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
         public List<URDFFilenameHolder> getFilenameHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }

      public static class URDFClip implements URDFItem
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
         public List<URDFFilenameHolder> getFilenameHolders()
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

   public static class URDFIMU implements URDFItem
   {
      private URDFIMUNoise noise;

      public URDFIMUNoise getNoise()
      {
         return noise;
      }

      @XmlElement(name = "noise")
      public void setNoise(URDFIMUNoise noise)
      {
         this.noise = noise;
      }

      @Override
      public String getContentAsString()
      {
         return format("[noise: %s]", noise);
      }

      @Override
      public List<URDFFilenameHolder> getFilenameHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class URDFIMUNoise implements URDFItem
      {
         public enum URDFIMUNoiseType
         {
            gaussian;

            public static URDFIMUNoiseType parse(String value)
            {
               if (value == null)
                  return null;
               for (URDFIMUNoiseType type : values())
               {
                  if (type.name().equals(value))
                     return type;
               }
               return null;
            }
         };

         private String type;
         private URDFNoiseParameters rate;
         private URDFNoiseParameters accel;

         public String getType()
         {
            return type;
         }

         public URDFNoiseParameters getRate()
         {
            return rate;
         }

         public URDFNoiseParameters getAccel()
         {
            return accel;
         }

         @XmlElement(name = "type")
         public void setType(String type)
         {
            this.type = type;
         }

         public void setType(URDFIMUNoiseType type)
         {
            setType(type.name());
         }

         @XmlElement(name = "rate")
         public void setRate(URDFNoiseParameters rate)
         {
            this.rate = rate;
         }

         @XmlElement(name = "accel")
         public void setAccel(URDFNoiseParameters accel)
         {
            this.accel = accel;
         }

         @Override
         public String getContentAsString()
         {
            return format("[type: %s, rate: %s, accel: %s]", type, rate, accel);
         }

         @Override
         public List<URDFFilenameHolder> getFilenameHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }

         public static class URDFNoiseParameters implements URDFItem
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
            public List<URDFFilenameHolder> getFilenameHolders()
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
