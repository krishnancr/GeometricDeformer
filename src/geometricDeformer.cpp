#include "geometricDeformer.h"

#include  <vector>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>



MTypeId     GeometricDeformer::id(0x00000256);
MObject     GeometricDeformer::a_inflPoint;
MObject     GeometricDeformer::a_inflRadii;
MObject     GeometricDeformer::a_compoundInfluences;
MObject     GeometricDeformer::a_compoundRadius;
MObject     GeometricDeformer::a_objectType;


struct InfluenceType
{
	static const int kSphere = 0;
	static const int kEllipsoid = 1;
};

GeometricDeformer::GeometricDeformer()
{

}


GeometricDeformer::~GeometricDeformer()
{
}


void* GeometricDeformer::creator()
{
	return new GeometricDeformer();
}


MStatus GeometricDeformer::deform(MDataBlock& data, MItGeometry& itGeo,
	const MMatrix& localToWorldMatrix, unsigned int geomIndex)
{
	MStatus status;

	MMatrix worldToLocalMatrix = localToWorldMatrix.inverse();
	for (; !itGeo.isDone(); itGeo.next())
	{
		MArrayDataHandle inputArray = data.inputArrayValue(a_compoundInfluences);
		unsigned int inputArrayCount = inputArray.elementCount();

		MArrayDataHandle radiusArray = data.inputArrayValue(a_compoundRadius);
		unsigned int radiusArrayCount = radiusArray.elementCount();

		MArrayDataHandle objTypeArray = data.inputArrayValue(a_objectType);
		unsigned int objTypeCount = objTypeArray.elementCount();

		if (radiusArrayCount != inputArrayCount)
		{
			return MStatus::kFailure;
		}

		std::vector<unsigned int> objectType(objTypeCount);
		for (unsigned int i = 0; i < objTypeCount; i++)
		{
			objectType[i] = objTypeArray.inputValue().asInt();
			objTypeArray.next();
		}

		for (unsigned int i = 0; i < inputArrayCount; i++)
		{

			MVector H = inputArray.inputValue().child(a_inflPoint).asFloat3();
			MPoint radius = radiusArray.inputValue().child(a_inflRadii).asFloat3();
			int objType = objectType[i];

			
			MPoint point = itGeo.position();
			MVector M = point * localToWorldMatrix;
			MVector u(M - H);
			double p = u.length();

			MPoint point_on_inflObj;

			if (objType == InfluenceType::kSphere)
			{
				// A sphere with a user defined radius
				point_on_inflObj = MPoint(radius.x, radius.y, radius.z,1.0f);
			}
			else if (objType == InfluenceType::kEllipsoid)
			{
				// Parametrization of ellipse with a couple of harcoded values for angles 
				float theta = 0.5;
				float gamma = 0.2;
				point_on_inflObj.x = radius.x * std::sin(gamma) * std::cos(theta);
				point_on_inflObj.y = radius.y * std::sin(gamma) * std::sin(theta);
				point_on_inflObj.z = radius.z * std::cos(gamma);
			}


			MPoint I = H + point_on_inflObj;
			double p0 = MVector(I - H).length();
			u.normalize();
			double p_prime = std::cbrt(std::pow(p0, 3) + std::pow(p, 3)) ;
			MPoint M_prime = H + p_prime * u;
			M_prime *= worldToLocalMatrix;
			itGeo.setPosition(M_prime);
			
			inputArray.next();
			radiusArray.next();

		}
	}

	return MS::kSuccess;
}


MStatus GeometricDeformer::initialize()
{
	MStatus status;
	// Attribute Initialization
	MFnNumericAttribute nAttr;
	MFnCompoundAttribute compAttr ;

	a_compoundInfluences = compAttr.create("influences", "influenes", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	a_inflPoint = nAttr.create("inflPoint", "inflPoint", MFnNumericData::k3Float);
	status = compAttr.addChild(a_inflPoint);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = compAttr.setArray(true);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = compAttr.setDisconnectBehavior(MFnAttribute::kDelete);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = addAttribute(a_compoundInfluences);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = attributeAffects(a_inflPoint, outputGeom);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = attributeAffects(a_compoundInfluences, outputGeom);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	a_compoundRadius = compAttr.create("radii", "radii", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	a_inflRadii = nAttr.create("influenceRadius", "inflRadius", MFnNumericData::k3Float);
	status = compAttr.addChild(a_inflRadii);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = compAttr.setArray(true);
	status = compAttr.setDisconnectBehavior(MFnAttribute::kDelete);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = addAttribute(a_compoundRadius);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = attributeAffects(a_compoundRadius, outputGeom);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = attributeAffects(a_inflRadii, outputGeom);
	CHECK_MSTATUS_AND_RETURN_IT(status);


	a_objectType = nAttr.create("objectType", "objType", MFnNumericData::kInt);
	status = nAttr.setArray(true);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = addAttribute(a_objectType);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = attributeAffects(a_objectType, outputGeom);
	CHECK_MSTATUS_AND_RETURN_IT(status);



	return status;
}