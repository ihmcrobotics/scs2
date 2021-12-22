package us.ihmc.scs2.definition;

import javax.xml.bind.annotation.XmlElement;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class AffineTransformDefinition extends AffineTransform
{
   public AffineTransformDefinition()
   {
      super();
   }

   public AffineTransformDefinition(AffineTransformReadOnly other)
   {
      super(other);
   }

   public AffineTransformDefinition(DMatrix linearTransform, Tuple3DReadOnly translation)
   {
      super(linearTransform, translation);
   }

   public AffineTransformDefinition(Matrix3DReadOnly linearTransform, Tuple3DReadOnly translation)
   {
      super(linearTransform, translation);
   }

   public AffineTransformDefinition(Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      super(orientation, translation);
   }

   public AffineTransformDefinition(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      super(rigidBodyTransform);
   }

   public AffineTransformDefinition(RotationMatrix rotationMatrix, Tuple3DReadOnly translation)
   {
      super(rotationMatrix, translation);
   }

   @XmlElement
   public void setM00(double m00)
   {
      getLinearTransform().setM00(m00);
   }

   @XmlElement
   public void setM01(double m01)
   {
      getLinearTransform().setM01(m01);
   }

   @XmlElement
   public void setM02(double m02)
   {
      getLinearTransform().setM02(m02);
   }

   @XmlElement
   public void setM03(double m03)
   {
      getTranslation().setX(m03);
   }

   @XmlElement
   public void setM10(double m10)
   {
      getLinearTransform().setM10(m10);
   }

   @XmlElement
   public void setM11(double m11)
   {
      getLinearTransform().setM11(m11);
   }

   @XmlElement
   public void setM12(double m12)
   {
      getLinearTransform().setM12(m12);
   }

   @XmlElement
   public void setM13(double m13)
   {
      getTranslation().setY(m13);
   }

   @XmlElement
   public void setM20(double m20)
   {
      getLinearTransform().setM20(m20);
   }

   @XmlElement
   public void setM21(double m21)
   {
      getLinearTransform().setM21(m21);
   }

   @XmlElement
   public void setM22(double m22)
   {
      getLinearTransform().setM22(m22);
   }

   @XmlElement
   public void setM23(double m23)
   {
      getTranslation().setZ(m23);
   }

   @Override
   public double getM00()
   {
      return super.getM00();
   }

   @Override
   public double getM01()
   {
      return super.getM01();
   }

   @Override
   public double getM02()
   {
      return super.getM02();
   }

   @Override
   public double getM03()
   {
      return super.getM03();
   }

   @Override
   public double getM10()
   {
      return super.getM10();
   }

   @Override
   public double getM11()
   {
      return super.getM11();
   }

   @Override
   public double getM12()
   {
      return super.getM12();
   }

   @Override
   public double getM13()
   {
      return super.getM13();
   }

   @Override
   public double getM20()
   {
      return super.getM20();
   }

   @Override
   public double getM21()
   {
      return super.getM21();
   }

   @Override
   public double getM22()
   {
      return super.getM22();
   }

   @Override
   public double getM23()
   {
      return super.getM23();
   }
}
