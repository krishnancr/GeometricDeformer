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
	
	MVector H = data.inputValue(a_inflPoint).asFloat3();
	float env = data.inputValue(envelope).asFloat();

		MPoint point;

		for (; !itGeo.isDone(); itGeo.next())
		{
			point = itGeo.position();
			MVector M = point * localToWorldMatrix;
			MVector u(M - H);
			double p = u.length();
			// Setting a hard coded value around the Influence Point just as a starting point
			MPoint I = H + MPoint(1.0f, 1.0f, 1.0f, 1.0f);
			double p0 = MVector(I - H).length();
			u.normalize();
			double p_prime = std::cbrt(std::pow(p0, 3) + std::pow(p, 3));
			MPoint M_prime = H + p_prime * u;
			M_prime *= worldToLocalMatrix;
			itGeo.setPosition(M_prime);
		}
	return MS::kSuccess;
}


MStatus GeometricDeformer::initialize()
{
	MStatus status;

	MFnMatrixAttribute mAttr;
	MFnNumericAttribute nAttr;
	MFnCompoundAttribute compAttr;

	
	a_inflPoint = nAttr.create("inflPoint", "inflPoint", MFnNumericData::k3Float);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	nAttr.setKeyable(true);
	addAttribute(a_inflPoint);
	
	attributeAffects(a_inflPoint, outputGeom);

	return status;
}