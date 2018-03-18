#include "geometricDeformer.h"

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>

MTypeId     GeometricDeformer::id(0x00000256);
MObject     GeometricDeformer::a_inflPoint;
MObject     GeometricDeformer::a_deformingMesh;


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

	MDataHandle deformData = data.inputValue(a_deformingMesh, &status);
	MObject dSurf = deformData.asMeshTransformed();
	MFnMesh fnDeformingMesh;
	fnDeformingMesh.setObject(dSurf);

	MMatrix worldToLocalMatrix = localToWorldMatrix.inverse();

	MVector H = data.inputValue(a_inflPoint).asFloat3();
	float env = data.inputValue(envelope).asFloat();

	for (int i = 0; i < fnDeformingMesh.numVertices(); i++)
	{
		MPoint aPoint;
		fnDeformingMesh.getPoint(i, aPoint, MSpace::kWorld);
		std::cerr << " Point is " << aPoint[0] << " " << aPoint[1] << " " << aPoint[2] << std::endl;
	


		MPoint point;

		for (; !itGeo.isDone(); itGeo.next())
		{
			point = itGeo.position();
			MVector M = point * localToWorldMatrix;
			MVector u(M - H);
			double p = u.length();
			// Setting a hard coded value around the Influence Point just as a starting point
			//MPoint I = H * 0.2;
			MVector I_blah(aPoint - H);
			double p0 = MVector(aPoint - H).length();
			u.normalize();
			double p_prime = std::cbrt(std::pow(p0, 3) + std::pow(p, 3));
			MPoint M_prime = H + p_prime * u;
			M_prime *= worldToLocalMatrix;
			itGeo.setPosition(M_prime);
		}

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

	MFnTypedAttribute typ_Attr;
	a_deformingMesh = typ_Attr.create("deformingMesh", "dm", MFnMeshData::kMesh);
	addAttribute(a_deformingMesh);
	attributeAffects(a_deformingMesh, outputGeom);



	return MS::kSuccess;
}