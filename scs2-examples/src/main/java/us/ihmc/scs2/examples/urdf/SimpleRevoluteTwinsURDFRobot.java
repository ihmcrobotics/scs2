package us.ihmc.scs2.examples.urdf;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import javax.xml.bind.JAXBException;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteTwinsJointReadOnly;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.simulation.robot.Robot;

public class SimpleRevoluteTwinsURDFRobot
{
   public static void main(String[] args) throws JAXBException
   {
      System.out.println(new Vector2D(0.2, 0.2).norm());

      InputStream is = SimpleCrossFourBarURDFRobot.class.getClassLoader().getResourceAsStream("urdf/SimpleRevoluteTwinsRobot.urdf");
      URDFModel urdfModel = URDFTools.loadURDFModel(is, Collections.emptyList(), SimpleCrossFourBarURDFRobot.class.getClassLoader());
      URDFTools.URDFParserProperties parserProperties = new URDFTools.URDFParserProperties();
      parserProperties.setRootJointFactory(null);
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel, parserProperties);
      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2(SimulationConstructionSet2.impulseBasedPhysicsEngineFactory());
      Robot robot = scs2.addRobot(robotDefinition);
      RevoluteTwinsJointReadOnly joint = robot.getAllJoints()
                                              .stream()
                                              .filter(j -> j instanceof RevoluteTwinsJointReadOnly)
                                              .map(RevoluteTwinsJointReadOnly.class::cast)
                                              .findFirst()
                                              .get();
      scs2.addYoGraphics(frameAfterJointGraphics(joint));
      scs2.addYoGraphics(bodyFixedFrameGraphics(joint));
      scs2.setCameraFocalPosition(0, 0, 0.3);
      scs2.setCameraPosition(0, 2, 0.3);
      scs2.startSimulationThread();
   }
   public static List<YoGraphicDefinition> frameAfterJointGraphics(RevoluteTwinsJointReadOnly joint)
   {
      List<YoGraphicDefinition> yoGraphics = new ArrayList<>();

      for (RevoluteJointReadOnly subJoint : Arrays.asList(joint.getJointA(), joint.getJointB()))
      {
         MovingReferenceFrame frameAfterJoint = subJoint.getFrameAfterJoint();
         yoGraphics.add(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(frameAfterJoint.getName(),
                                                                                  new FramePose3D(frameAfterJoint),
                                                                                  0.12,
                                                                                  ColorDefinitions.DodgerBlue()));
      }
      return yoGraphics;
   }

   public static List<YoGraphicDefinition> bodyFixedFrameGraphics(RevoluteTwinsJointReadOnly joint)
   {
      List<YoGraphicDefinition> yoGraphics = new ArrayList<>();

      List<MovingReferenceFrame> frames = Arrays.asList(joint.getPredecessor().getBodyFixedFrame(),
                                                        joint.getSuccessor().getBodyFixedFrame(),
                                                        joint.getBodyAB().getBodyFixedFrame());

      for (MovingReferenceFrame frame : frames)
      {
         yoGraphics.add(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(frame.getName(),
                                                                                  new FramePose3D(frame),
                                                                                  0.10,
                                                                                  ColorDefinitions.Fuchsia()));
      }
      return yoGraphics;
   }
}
