#include "geometricDeformer.h"

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
		MPoint I = H * 1.2;
		MVector I_blah(I - H);
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
	a_inflPoint = nAttr.create("inflPoint", "inflPoint", MFnNumericData::k3Float);
	nAttr.setKeyable(true);
	addAttribute(a_inflPoint);
	attributeAffects(a_inflPoint, outputGeom);

	return MS::kSuccess;
}