#include "geometricDeformer.h"

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
MObject     GeometricDeformer::a_compoundInfluences;



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
		for (unsigned int i = 0; i < inputArrayCount; i++)
		{
			MDataHandle targetElement = inputArray.inputValue();
			MVector H = targetElement.child(a_inflPoint).asFloat3();
			MPoint point = itGeo.position();
			MVector M = point * localToWorldMatrix;
			MVector u(M - H);
			double p = u.length();
			// Setting a hard coded value around the Influence Point just as a starting point
			MPoint I = H + MPoint(1.0f, 1.0f, 1.0f, 1.0f);
			double p0 = MVector(I - H).length();
			u.normalize();
			double p_prime = std::cbrt(std::pow(p0, 3) + std::pow(p, 3)) ;
			MPoint M_prime = H + p_prime * u;
			M_prime *= worldToLocalMatrix;
			itGeo.setPosition(M_prime);
			inputArray.next();
		}
	}

	return MS::kSuccess;
}


MStatus GeometricDeformer::initialize()
{
	MStatus status;

	// Attribute Initialization
	MFnMatrixAttribute mAttr;
	MFnNumericAttribute nAttr;
	MFnCompoundAttribute compAttr;

	
	a_compoundInfluences = compAttr.create("influences", "influenes", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	a_inflPoint = nAttr.create("inflPoint", "inflPoint", MFnNumericData::k3Float);
	CHECK_MSTATUS_AND_RETURN_IT(status);

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

	return status;
}