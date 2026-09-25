package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * Contact properties for one named group of collision shapes, emitted as a MuJoCo
 * {@code <default class="...">} block.
 *
 * <p>Every field starts unset and an unset field is simply not emitted, so the geom inherits it
 * from the model-wide values in {@link MujocoSimulationParametersReadOnly}. Set only what differs:
 * a foot class that overrides friction and priority, for instance, still inherits the model's
 * solref and solimp.
 *
 * <p>{@link #setPriority(int)} is the one that is easy to miss. When two geoms touch and their
 * priorities are equal, MuJoCo combines their parameters -- elementwise maximum for friction, the
 * harder of the two for solref. Giving the foot a higher priority than the terrain makes the foot's
 * numbers win outright, which is usually what "tune the foot contact" is meant to achieve.
 */
public class MujocoContactProperties
{
   private double friction_slide = Double.NaN;
   private double friction_spin = Double.NaN;
   private double friction_roll = Double.NaN;
   private double solref_timeconst = Double.NaN;
   private double solref_dampratio = Double.NaN;
   private double solimp_dmin = Double.NaN;
   private double solimp_dmax = Double.NaN;
   private double solimp_width = Double.NaN;
   private double solimp_midpoint = Double.NaN;
   private double solimp_power = Double.NaN;
   private double margin = Double.NaN;
   private double gap = Double.NaN;
   private int condim = -1;
   private int priority = -1;

   public MujocoContactProperties()
   {
   }

   public MujocoContactProperties(MujocoContactProperties other)
   {
      set(other);
   }

   public void set(MujocoContactProperties other)
   {
      friction_slide = other.friction_slide;
      friction_spin = other.friction_spin;
      friction_roll = other.friction_roll;
      solref_timeconst = other.solref_timeconst;
      solref_dampratio = other.solref_dampratio;
      solimp_dmin = other.solimp_dmin;
      solimp_dmax = other.solimp_dmax;
      solimp_width = other.solimp_width;
      solimp_midpoint = other.solimp_midpoint;
      solimp_power = other.solimp_power;
      margin = other.margin;
      gap = other.gap;
      condim = other.condim;
      priority = other.priority;
   }

   /**
    * Sets all three friction coefficients: sliding, torsional and rolling.
    *
    * <p>Torsional friction is the one worth measuring rather than guessing. A box foot on a plane
    * already makes several corner contacts, so most of its yaw resistance comes from sliding
    * friction at those corners; adding a large torsional coefficient on top double-counts it.
    * MuJoCo's default is 0.005 and values in the 0.005 to 0.02 range are the usual starting point.
    */
   public void setFriction(double slide, double spin, double roll)
   {
      friction_slide = slide;
      friction_spin = spin;
      friction_roll = roll;
   }

   /** @see #setFriction(double, double, double) */
   public void setFrictionSlide(double friction_slide)
   {
      this.friction_slide = friction_slide;
   }

   /** @see #setFriction(double, double, double) */
   public void setFrictionSpin(double friction_spin)
   {
      this.friction_spin = friction_spin;
   }

   /** @see #setFriction(double, double, double) */
   public void setFrictionRoll(double friction_roll)
   {
      this.friction_roll = friction_roll;
   }

   /**
    * Contact settling time constant and damping ratio. Keep the time constant at or above twice the
    * simulation timestep; below that the contact is stiffer than the integrator can follow.
    */
   public void setSolref(double timeconst, double dampratio)
   {
      solref_timeconst = timeconst;
      solref_dampratio = dampratio;
   }

   /** Contact impedance profile: minimum and maximum hardness, transition width, midpoint and power. */
   public void setSolimp(double dmin, double dmax, double width, double midpoint, double power)
   {
      solimp_dmin = dmin;
      solimp_dmax = dmax;
      solimp_width = width;
      solimp_midpoint = midpoint;
      solimp_power = power;
   }

   /**
    * Contact dimensionality: 1 normal only, 3 adds sliding friction, 4 adds torsional friction,
    * 6 adds rolling friction.
    */
   public void setCondim(int condim)
   {
      this.condim = condim;
   }

   /** Distance at which contacts are generated, and the part of that distance that carries no force. */
   public void setMarginAndGap(double margin, double gap)
   {
      this.margin = margin;
      this.gap = gap;
   }

   /**
    * When two geoms touch, the higher priority one's friction and solref are used outright instead
    * of being combined. Default (unset) leaves MuJoCo's 0, i.e. combine.
    */
   public void setPriority(int priority)
   {
      this.priority = priority;
   }

   /**
    * Appends this class's {@code <geom .../>} attributes. Unset fields are omitted so the geom
    * inherits them. Returns false when nothing at all is set, so the caller can skip the block.
    *
    * <p>Public for the MJCF builder rather than for callers setting up a simulation.
    */
   public boolean appendGeomAttributes(StringBuilder sb)
   {
      int lengthBefore = sb.length();
      if (isSet(friction_slide) || isSet(friction_spin) || isSet(friction_roll))
      {
         // MuJoCo's friction attribute is positional, so a partially specified class has to fill the
         // gaps with MuJoCo's own defaults rather than leave them out.
         sb.append(" friction=\"").append(orDefault(friction_slide, 1.0))
           .append(' ').append(orDefault(friction_spin, 0.005))
           .append(' ').append(orDefault(friction_roll, 0.0001)).append('"');
      }
      if (isSet(solref_timeconst) || isSet(solref_dampratio))
      {
         sb.append(" solref=\"").append(orDefault(solref_timeconst, 0.02))
           .append(' ').append(orDefault(solref_dampratio, 1.0)).append('"');
      }
      if (isSet(solimp_dmin) || isSet(solimp_dmax) || isSet(solimp_width) || isSet(solimp_midpoint) || isSet(solimp_power))
      {
         sb.append(" solimp=\"").append(orDefault(solimp_dmin, 0.9))
           .append(' ').append(orDefault(solimp_dmax, 0.95))
           .append(' ').append(orDefault(solimp_width, 0.001))
           .append(' ').append(orDefault(solimp_midpoint, 0.5))
           .append(' ').append(orDefault(solimp_power, 2.0)).append('"');
      }
      if (condim >= 0)
         sb.append(" condim=\"").append(condim).append('"');
      if (isSet(margin))
         sb.append(" margin=\"").append(margin).append('"');
      if (isSet(gap))
         sb.append(" gap=\"").append(gap).append('"');
      if (priority >= 0)
         sb.append(" priority=\"").append(priority).append('"');
      return sb.length() > lengthBefore;
   }

   private static boolean isSet(double value)
   {
      return !Double.isNaN(value);
   }

   private static double orDefault(double value, double fallback)
   {
      return Double.isNaN(value) ? fallback : value;
   }
}
