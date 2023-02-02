package us.ihmc.scs2.session.tools;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoQuaternionDefinition;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoTuple2DDefinition;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoTuple3DDefinition;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoYawPitchRollDefinition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicText;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicText3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicTriangle;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVRML;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLine2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicArrow3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCoordinateSystem3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCylinder3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicLine2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygon2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygonExtruded3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolynomial3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoListDefinition;
import us.ihmc.yoVariables.variable.YoVariable;

public class SCS1GraphicConversionTools
{
   public static final String WORLD_FRAME = "worldFrame";

   private static final String GRAPHIC_3D_SPHERE_NAME = "Sphere";

   public static List<YoGraphicDefinition> toYoGraphicDefinitions(YoGraphicsListRegistry registry)
   {
      List<YoGraphicsList> yoGraphicsLists = new ArrayList<>();
      registry.getRegisteredYoGraphicsLists(yoGraphicsLists);
      List<ArtifactList> artifactLists = new ArrayList<>();
      registry.getRegisteredArtifactLists(artifactLists);

      Map<String, YoGraphicGroupDefinition> definitionMap = new LinkedHashMap<>();

      for (YoGraphicsList yoGraphicsList : yoGraphicsLists)
      {
         YoGraphicGroupDefinition newDefinition = toYoGraphicGroupDefinition(yoGraphicsList);
         YoGraphicGroupDefinition oldDefinition = definitionMap.get(newDefinition.getName());
         if (oldDefinition != null)
            oldDefinition.getChildren().addAll(newDefinition.getChildren());
         else
            definitionMap.put(newDefinition.getName(), newDefinition);
      }
      for (ArtifactList artifactList : artifactLists)
      {
         YoGraphicGroupDefinition newDefinition = toYoGraphicGroupDefinition(artifactList);
         YoGraphicGroupDefinition oldDefinition = definitionMap.get(newDefinition.getName());
         if (oldDefinition != null)
            oldDefinition.getChildren().addAll(newDefinition.getChildren());
         else
            definitionMap.put(newDefinition.getName(), newDefinition);
      }

      return new ArrayList<>(definitionMap.values());
   }

   public static YoGraphicGroupDefinition toYoGraphicGroupDefinition(YoGraphicsList yoGraphicsList)
   {
      YoGraphicGroupDefinition groupDefinition = new YoGraphicGroupDefinition();
      groupDefinition.setName(yoGraphicsList.getLabel());
      groupDefinition.setChildren(yoGraphicsList.getYoGraphics().stream().map(SCS1GraphicConversionTools::toYoGraphicDefinition).collect(Collectors.toList()));
      return groupDefinition;
   }

   public static YoGraphicGroupDefinition toYoGraphicGroupDefinition(ArtifactList artifactList)
   {
      YoGraphicGroupDefinition groupDefinition = new YoGraphicGroupDefinition();
      groupDefinition.setName(artifactList.getLabel());
      groupDefinition.setChildren(artifactList.getArtifacts().stream().map(SCS1GraphicConversionTools::toYoGraphicDefinition).collect(Collectors.toList()));
      return groupDefinition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphic yoGraphic)
   {
      if (yoGraphic == null)
         return null;

      YoGraphicDefinition definition = null;
      if (yoGraphic instanceof YoGraphicPolygon)
         definition = toYoGraphicDefinition((YoGraphicPolygon) yoGraphic);
      else if (yoGraphic instanceof YoGraphicShape)
         definition = toYoGraphicDefinition((YoGraphicShape) yoGraphic);
      else if (yoGraphic instanceof YoGraphicText)
         definition = toYoGraphicDefinition((YoGraphicText) yoGraphic);
      else if (yoGraphic instanceof YoGraphicText3D)
         definition = toYoGraphicDefinition((YoGraphicText3D) yoGraphic);
      else if (yoGraphic instanceof YoGraphicCoordinateSystem)
         definition = toYoGraphicDefinition((YoGraphicCoordinateSystem) yoGraphic);
      else if (yoGraphic instanceof YoGraphicCylinder)
         definition = toYoGraphicDefinition((YoGraphicCylinder) yoGraphic);
      else if (yoGraphic instanceof YoGraphicPolygon3D)
         definition = toYoGraphicDefinition((YoGraphicPolygon3D) yoGraphic);
      else if (yoGraphic instanceof YoGraphicPolynomial3D)
         definition = toYoGraphicDefinition((YoGraphicPolynomial3D) yoGraphic);
      else if (yoGraphic instanceof YoGraphicPosition)
         definition = toYoGraphicDefinition((YoGraphicPosition) yoGraphic);
      else if (yoGraphic instanceof YoGraphicTriangle)
         definition = toYoGraphicDefinition((YoGraphicTriangle) yoGraphic);
      else if (yoGraphic instanceof YoGraphicVector)
         definition = toYoGraphicDefinition((YoGraphicVector) yoGraphic);
      else
      {
         LogTools.error("Unsupported YoGraphic type: [{}]={}", yoGraphic.getClass().getName(), yoGraphic);
         return null;
      }

      if (definition != null)
      {
         definition.setName(yoGraphic.getName());
         definition.setVisible(yoGraphic.isGraphicObjectShowing());
      }
      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicPolygon yoGraphicPolygon)
   {
      YoGraphicPolygonExtruded3DDefinition definition = new YoGraphicPolygonExtruded3DDefinition();
      definition.setName(yoGraphicPolygon.getName());
      YoVariable[] yoVariables = yoGraphicPolygon.getVariables();
      int yoVariableIndex = 0;
      double[] constants = yoGraphicPolygon.getConstants();
      int constantIndex = 1; // 0 corresponds to the scale factor

      int vertexBufferSize = (int) constants[constantIndex++];
      definition.setThickness(constants.length == 3 ? constants[constantIndex++] : 0.01);

      definition.setNumberOfVertices(yoVariables[yoVariableIndex++].getFullNameString());

      List<YoTuple2DDefinition> vertices = new ArrayList<>();

      for (int i = 0; i < vertexBufferSize; i++)
      {
         YoTuple2DDefinition vertex = newYoTuple2DDefinition(yoVariables, yoVariableIndex);
         vertex.setReferenceFrame(null);
         vertices.add(vertex);
         yoVariableIndex += 2;
      }

      definition.setVertices(vertices);

      definition.setPosition(newYoTuple3DDefinition(yoVariables, yoVariableIndex));
      yoVariableIndex += 3;

      if (yoGraphicPolygon.isUsingYawPitchRoll())
      {
         definition.setOrientation(newYoYawPitchRollDefinition(yoVariables, yoVariableIndex));
         yoVariableIndex += 3;
      }
      else
      {
         definition.setOrientation(newYoQuaternionDefinition(yoVariables, yoVariableIndex));
         yoVariableIndex += 4;
      }

      definition.setColor(toColorDefinition(yoGraphicPolygon.getAppearance()));
      definition.setVisible(yoGraphicPolygon.isGraphicObjectShowing());

      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicShape yoGraphicShape)
   {
      if (yoGraphicShape == null)
         return null;

      // TODO Unsupported for now, need to convert Graphics3DObject
      LogTools.warn("Unsupported YoGraphic type: {}, yoGraphic: {}", yoGraphicShape.getClass().getSimpleName(), yoGraphicShape);
      return null;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicText yoGraphicText)
   {
      if (yoGraphicText == null)
         return null;

      // TODO Unsupported for now
      LogTools.warn("Unsupported YoGraphic type: {}, yoGraphic: {}", yoGraphicText.getClass().getSimpleName(), yoGraphicText);
      return null;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicText3D yoGraphicText3D)
   {
      if (yoGraphicText3D == null)
         return null;

      // TODO Unsupported for now
      LogTools.warn("Unsupported YoGraphic type: {}, yoGraphic: {}", yoGraphicText3D.getClass().getSimpleName(), yoGraphicText3D);
      return null;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicCoordinateSystem yoGraphicCoordinateSystem)
   {
      if (yoGraphicCoordinateSystem instanceof YoGraphicEllipsoid)
         return toYoGraphicDefinition((YoGraphicEllipsoid) yoGraphicCoordinateSystem);
      // TODO This is loaded the same way as a YoGraphicCoordinateSystem
      //      if (yoGraphicCoordinateSystem instanceof YoGraphicReferenceFrame)
      //         return toYoGraphicDefinition((YoGraphicReferenceFrame) yoGraphicCoordinateSystem);
      if (yoGraphicCoordinateSystem instanceof YoGraphicVRML)
         return toYoGraphicDefinition((YoGraphicVRML) yoGraphicCoordinateSystem);

      YoGraphicCoordinateSystem3DDefinition definition = new YoGraphicCoordinateSystem3DDefinition();
      definition.setName(yoGraphicCoordinateSystem.getName());

      YoVariable[] yoVariables = yoGraphicCoordinateSystem.getVariables();
      int yoVariableIndex = 0;
      double[] constants = yoGraphicCoordinateSystem.getConstants();

      definition.setPosition(newYoTuple3DDefinition(yoVariables, yoVariableIndex));
      yoVariableIndex += 3;

      if (yoVariables.length == 6)
      {
         definition.setOrientation(newYoYawPitchRollDefinition(yoVariables, yoVariableIndex));
         yoVariableIndex += 3;
      }
      else
      {
         definition.setOrientation(newYoQuaternionDefinition(yoVariables, yoVariableIndex));
         yoVariableIndex += 4;
      }

      double scale = constants[0];
      definition.setBodyLength(0.90 * scale);
      definition.setHeadLength(0.10 * scale);
      definition.setBodyRadius(0.02 * scale);
      definition.setHeadRadius(0.04 * scale);

      if (constants.length == 3)
      {
         definition.setColor(ColorDefinition.rgb((int) constants[1]));
         definition.getColor().setAlpha(1.0 - constants[2]);
      }
      definition.setVisible(yoGraphicCoordinateSystem.isGraphicObjectShowing());

      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicEllipsoid yoGraphicEllipsoid)
   {
      // TODO Unsupported for now
      return null;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicReferenceFrame yoGraphicReferenceFrame)
   {
      return toYoGraphicDefinition((YoGraphicCoordinateSystem) yoGraphicReferenceFrame);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicVRML yoGraphicVRML)
   {
      // TODO Unsupported for now
      return null;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicCylinder yoGraphicCylinder)
   {
      YoGraphicCylinder3DDefinition definition = new YoGraphicCylinder3DDefinition();
      definition.setName(yoGraphicCylinder.getName());

      YoVariable[] yoVariables = yoGraphicCylinder.getVariables();
      int yoVariableIndex = 0;
      double[] constants = yoGraphicCylinder.getConstants();

      // FIXME the YoGraphicCylinder only gives the position of the base and not the center.
      definition.setCenter(newYoTuple3DDefinition(yoVariables, yoVariableIndex));
      yoVariableIndex += 3;
      definition.setAxis(newYoTuple3DDefinition(yoVariables, yoVariableIndex));
      yoVariableIndex += 3;

      definition.setRadius(constants[0]);
      definition.setColor(toColorDefinition(yoGraphicCylinder.getAppearance()));
      definition.setVisible(yoGraphicCylinder.isGraphicObjectShowing());

      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicPolygon3D yoGraphicPolygon3D)
   {
      // TODO Unsupported for now
      return null;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicPolynomial3D yoGraphicPolynomial3D)
   {
      YoVariable[] yoVariables = yoGraphicPolynomial3D.getVariables();
      int yoVariableIndex = 0;
      double[] constants = yoGraphicPolynomial3D.getConstants();

      boolean hasPoseDefined = (int) constants[3] == 1;

      if (hasPoseDefined)
      {
         LogTools.error("Unsupported feature [hasPoseDefined] for " + yoGraphicPolynomial3D);
         return null;
      }

      int numberOfPolynomials = (int) constants[4];
      double[] polynomialSizes = Arrays.copyOfRange(constants, 5, 5 + 3 * numberOfPolynomials);

      if (numberOfPolynomials == 1)
      {
         YoGraphicPolynomial3DDefinition definition = new YoGraphicPolynomial3DDefinition();
         definition.setName(yoGraphicPolynomial3D.getName());
         definition.setSize(constants[0]);
         definition.setTimeResolution((int) constants[1]);
         definition.setNumberOfDivisions((int) constants[2]);

         int xSize = (int) polynomialSizes[0];
         String xNumberOfCoeffs = yoVariables[yoVariableIndex].getFullNameString();
         List<String> xCoeffs = Stream.of(Arrays.copyOfRange(yoVariables, yoVariableIndex + 1, yoVariableIndex + 1 + xSize - 1))
                                      .map(YoVariable::getFullNameString)
                                      .collect(Collectors.toList());
         yoVariableIndex += xSize;

         int ySize = (int) polynomialSizes[1];
         String yNumberOfCoeffs = yoVariables[yoVariableIndex].getFullNameString();
         List<String> yCoeffs = Stream.of(Arrays.copyOfRange(yoVariables, yoVariableIndex + 1, yoVariableIndex + 1 + ySize - 1))
                                      .map(YoVariable::getFullNameString)
                                      .collect(Collectors.toList());
         yoVariableIndex += ySize;

         int zSize = (int) polynomialSizes[2];
         String zNumberOfCoeffs = yoVariables[yoVariableIndex].getFullNameString();
         List<String> zCoeffs = Stream.of(Arrays.copyOfRange(yoVariables, yoVariableIndex + 1, yoVariableIndex + 1 + zSize - 1))
                                      .map(YoVariable::getFullNameString)
                                      .collect(Collectors.toList());
         yoVariableIndex += ySize;

         definition.setCoefficientsX(new YoListDefinition(xCoeffs, xNumberOfCoeffs));
         definition.setCoefficientsY(new YoListDefinition(yCoeffs, yNumberOfCoeffs));
         definition.setCoefficientsZ(new YoListDefinition(zCoeffs, zNumberOfCoeffs));
         definition.setStartTime(0.0);
         definition.setEndTime(yoVariables[yoVariableIndex].getFullNameString());
         definition.setVisible(yoGraphicPolynomial3D.isGraphicObjectShowing());
         return definition;
      }
      else
      {
         YoGraphicGroupDefinition groupDefinition = new YoGraphicGroupDefinition();
         groupDefinition.setName(yoGraphicPolynomial3D.getName());
         groupDefinition.setChildren(new ArrayList<>());

         YoVariable[] waypointTimes = Arrays.copyOfRange(yoVariables, yoVariables.length - 3 - numberOfPolynomials, yoVariables.length - 3 + 1);

         for (int i = 0; i < numberOfPolynomials; i++)
         {
            YoGraphicPolynomial3DDefinition definition = new YoGraphicPolynomial3DDefinition();
            definition.setName(yoGraphicPolynomial3D.getName() + Integer.toString(i));
            definition.setSize(constants[0]);
            definition.setTimeResolution((int) constants[1]);
            definition.setNumberOfDivisions((int) constants[2]);

            int xSize = (int) polynomialSizes[0];
            String xNumberOfCoeffs = yoVariables[yoVariableIndex].getFullNameString();
            List<String> xCoeffs = Stream.of(Arrays.copyOfRange(yoVariables, yoVariableIndex + 1, yoVariableIndex + 1 + xSize - 1))
                                         .map(YoVariable::getFullNameString)
                                         .collect(Collectors.toList());
            yoVariableIndex += xSize;

            int ySize = (int) polynomialSizes[1];
            String yNumberOfCoeffs = yoVariables[yoVariableIndex].getFullNameString();
            List<String> yCoeffs = Stream.of(Arrays.copyOfRange(yoVariables, yoVariableIndex + 1, yoVariableIndex + 1 + ySize - 1))
                                         .map(YoVariable::getFullNameString)
                                         .collect(Collectors.toList());
            yoVariableIndex += ySize;

            int zSize = (int) polynomialSizes[2];
            String zNumberOfCoeffs = yoVariables[yoVariableIndex].getFullNameString();
            List<String> zCoeffs = Stream.of(Arrays.copyOfRange(yoVariables, yoVariableIndex + 1, yoVariableIndex + 1 + zSize - 1))
                                         .map(YoVariable::getFullNameString)
                                         .collect(Collectors.toList());
            yoVariableIndex += ySize;

            definition.setCoefficientsX(new YoListDefinition(xCoeffs, xNumberOfCoeffs));
            definition.setCoefficientsY(new YoListDefinition(yCoeffs, yNumberOfCoeffs));
            definition.setCoefficientsZ(new YoListDefinition(zCoeffs, zNumberOfCoeffs));
            definition.setStartTime(i == 0 ? Double.toString(0.0) : waypointTimes[i - 1].getFullNameString());
            definition.setEndTime(waypointTimes[i].getFullNameString());
            definition.setVisible(yoGraphicPolynomial3D.isGraphicObjectShowing());
            groupDefinition.getChildren().add(definition);
         }

         return groupDefinition;
      }
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicPosition yoGraphicPosition)
   {
      YoGraphicPoint3DDefinition definition = new YoGraphicPoint3DDefinition();
      definition.setName(yoGraphicPosition.getName());

      YoVariable[] yoVariables = yoGraphicPosition.getVariables();
      double[] constants = yoGraphicPosition.getConstants();

      YoTuple3DDefinition position = new YoTuple3DDefinition();
      position.setX(yoVariables[0].getFullNameString());
      position.setY(yoVariables[1].getFullNameString());
      position.setZ(yoVariables.length == 3 ? yoVariables[2].getFullNameString() : Double.toString(0.0));
      position.setReferenceFrame(WORLD_FRAME);
      definition.setPosition(position);
      definition.setSize(constants[0]);
      definition.setGraphicName(GRAPHIC_3D_SPHERE_NAME);
      definition.setColor(toColorDefinition(yoGraphicPosition.getAppearance()));
      definition.setVisible(yoGraphicPosition.isGraphicObjectShowing());
      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicTriangle yoGraphicTriangle)
   {
      // TODO Unupported for now
      return null;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicVector yoGraphicVector)
   {
      if (yoGraphicVector instanceof YoGraphicLineSegment)
         return toYoGraphicDefinition((YoGraphicLineSegment) yoGraphicVector);

      YoGraphicArrow3DDefinition definition = new YoGraphicArrow3DDefinition();
      definition.setName(yoGraphicVector.getName());

      YoVariable[] yoVariables = yoGraphicVector.getVariables();
      double[] constants = yoGraphicVector.getConstants();

      definition.setOrigin(newYoTuple3DDefinition(yoVariables, 0));
      definition.setDirection(newYoTuple3DDefinition(yoVariables, 3));

      double scale = constants[0];
      definition.setBodyLength(0.9000 * scale);
      definition.setHeadLength(0.1000 * scale);
      definition.setBodyRadius(0.0150 * scale);
      definition.setHeadRadius(0.0375 * scale);
      definition.setScaleLength(true);
      definition.setScaleRadius(true);
      definition.setColor(toColorDefinition(yoGraphicVector.getAppearance()));
      definition.setVisible(yoGraphicVector.isGraphicObjectShowing());
      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicLineSegment yoGraphicLineSegment)
   {
      // TODO Unsupported for now
      return null;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(Artifact artifact)
   {
      if (artifact == null)
         return null;

      YoGraphicDefinition definition = null;

      if (artifact instanceof YoArtifactLine2d)
         definition = toYoGraphicDefinition((YoArtifactLine2d) artifact);
      else if (artifact instanceof YoArtifactLineSegment2d)
         definition = toYoGraphicDefinition((YoArtifactLineSegment2d) artifact);
      else if (artifact instanceof YoArtifactOval)
         definition = toYoGraphicDefinition((YoArtifactOval) artifact);
      else if (artifact instanceof YoArtifactPolygon)
         definition = toYoGraphicDefinition((YoArtifactPolygon) artifact);
      else if (artifact instanceof YoArtifactPosition)
         definition = toYoGraphicDefinition((YoArtifactPosition) artifact);
      else
      {
         LogTools.error("Unsupported YoArtifact type: " + artifact);
         return null;
      }

      definition.setName(artifact.getID());
      definition.setVisible(artifact.isVisible());
      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactLine2d yoArtifactLine2d)
   {
      YoGraphicLine2DDefinition definition = new YoGraphicLine2DDefinition();
      definition.setName(yoArtifactLine2d.getName());

      YoVariable[] yoVariables = yoArtifactLine2d.getVariables();
      definition.setOrigin(newYoTuple2DDefinition(yoVariables, 0));
      definition.setDirection(newYoTuple2DDefinition(yoVariables, 2));
      definition.setStrokeColor(toColorDefinition(yoArtifactLine2d.getAppearance()));
      definition.setStrokeWidth(1.5);
      definition.setVisible(yoArtifactLine2d.isVisible());
      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactLineSegment2d yoArtifactLineSegment2d)
   {
      YoGraphicLine2DDefinition definition = new YoGraphicLine2DDefinition();
      definition.setName(yoArtifactLineSegment2d.getName());

      YoVariable[] yoVariables = yoArtifactLineSegment2d.getVariables();
      definition.setOrigin(newYoTuple2DDefinition(yoVariables, 0));
      definition.setDestination(newYoTuple2DDefinition(yoVariables, 2));
      definition.setStrokeColor(toColorDefinition(yoArtifactLineSegment2d.getAppearance()));
      definition.setStrokeWidth(1.5);
      definition.setVisible(yoArtifactLineSegment2d.isVisible());
      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactOval yoArtifactOval)
   {
      // TODO Unsupported for now
      return null;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactPolygon yoArtifactPolygon)
   {
      YoGraphicPolygon2DDefinition definition = new YoGraphicPolygon2DDefinition();
      definition.setName(yoArtifactPolygon.getName());

      YoVariable[] yoVariables = yoArtifactPolygon.getVariables();
      int yoVariableIndex = 0;
      definition.setNumberOfVertices(yoVariables[yoVariableIndex++].getFullNameString());
      definition.setVertices(new ArrayList<>());

      for (int i = 1; i < yoVariables.length; i += 2)
      {
         definition.getVertices().add(newYoTuple2DDefinition(yoVariables, i));
      }

      if (yoArtifactPolygon.getConstants()[0] == 1.0)
         definition.setFillColor(toColorDefinition(yoArtifactPolygon.getAppearance()));
      definition.setStrokeColor(toColorDefinition(yoArtifactPolygon.getAppearance()));
      definition.setStrokeWidth(1.5);
      definition.setVisible(yoArtifactPolygon.isVisible());
      return definition;
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactPosition yoArtifactPosition)
   {
      YoGraphicPoint2DDefinition definition = new YoGraphicPoint2DDefinition();
      definition.setName(yoArtifactPosition.getName());
      definition.setPosition(newYoTuple2DDefinition(yoArtifactPosition.getVariables(), 0));
      definition.setSize(2.0 * yoArtifactPosition.getConstants()[0]);

      boolean fill = false;
      int graphicTypeIndex = (int) yoArtifactPosition.getConstants()[1];

      if (graphicTypeIndex >= 0 && graphicTypeIndex < GraphicType.values().length)
      {
         DefaultPoint2DGraphic graphic = null;
         switch (GraphicType.values()[graphicTypeIndex])
         {
            case BALL:
               graphic = DefaultPoint2DGraphic.CIRCLE;
               break;
            case SOLID_BALL:
               graphic = DefaultPoint2DGraphic.CIRCLE;
               fill = true;
               break;
            case CROSS:
               graphic = DefaultPoint2DGraphic.CROSS;
               break;
            case BALL_WITH_CROSS:
               graphic = DefaultPoint2DGraphic.CIRCLE_PLUS;
               break;
            case ROTATED_CROSS:
               graphic = DefaultPoint2DGraphic.CROSS;
               break;
            case BALL_WITH_ROTATED_CROSS:
               graphic = DefaultPoint2DGraphic.CIRCLE_CROSS;
               break;
            case DIAMOND:
               graphic = DefaultPoint2DGraphic.DIAMOND;
               break;
            case DIAMOND_WITH_CROSS:
               graphic = DefaultPoint2DGraphic.DIAMOND_PLUS;
               break;
            case SQUARE:
               graphic = DefaultPoint2DGraphic.SQUARE;
               break;
            case SQUARE_WITH_CROSS:
               graphic = DefaultPoint2DGraphic.SQUARE_CROSS;
               break;
            case ELLIPSOID:
               graphic = null;
               break;
         }

         definition.setGraphicName(graphic != null ? graphic.getGraphicName() : null);
      }

      ColorDefinition color = toColorDefinition(yoArtifactPosition.getAppearance());

      if (fill)
      {
         definition.setFillColor(color);
         definition.setStrokeWidth(0.0);
      }
      else
      {
         definition.setStrokeColor(color);
         definition.setStrokeWidth(1.5);
      }

      definition.setVisible(yoArtifactPosition.isVisible());
      return definition;
   }

   public static ColorDefinition toColorDefinition(AppearanceDefinition appearanceDefinition)
   {
      ColorDefinition definition = new ColorDefinition();
      definition.setAlpha(1.0 - appearanceDefinition.getTransparency());

      if (appearanceDefinition instanceof YoAppearanceMaterial)
      {
         YoAppearanceMaterial yoAppearanceMaterial = (YoAppearanceMaterial) appearanceDefinition;
         definition.setRed(yoAppearanceMaterial.getDiffuseColor().getX());
         definition.setGreen(yoAppearanceMaterial.getDiffuseColor().getY());
         definition.setBlue(yoAppearanceMaterial.getDiffuseColor().getZ());
      }
      else if (appearanceDefinition instanceof YoAppearanceRGBColor)
      {
         YoAppearanceRGBColor yoAppearanceRGBColor = (YoAppearanceRGBColor) appearanceDefinition;
         definition.setRed(yoAppearanceRGBColor.getRed());
         definition.setGreen(yoAppearanceRGBColor.getGreen());
         definition.setBlue(yoAppearanceRGBColor.getBlue());
      }
      else
      {
         LogTools.error("Unsupported appearance: " + appearanceDefinition);
      }

      return definition;
   }
}
