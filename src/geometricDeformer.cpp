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
#include <maya/MFloatPointArray.h>
#include <maya/MStreamUtils.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/task_scheduler_init.h>


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


struct parallel_deform {
	parallel_deform
	(
		const std::vector<unsigned int> influenceObject,
		const std::vector <MPoint> infuenceRadius,
		const std::vector <MVector> toolCentre,
		const MMatrix worldToLocalMatrix,
		const MMatrix localToWorldMatrix,
		MFloatPointArray& meshPoints
	) :
		m_influenceObject(influenceObject),
		m_infuenceRadius(infuenceRadius),
		m_toolCentre(toolCentre),
		m_worldToLocalMatrix(worldToLocalMatrix),
		m_localToWorldMatrix(localToWorldMatrix),
		m_meshPoints(meshPoints)
	{}
	
	void operator()(const tbb::blocked_range<size_t>& r) const
	{
		for (size_t i = r.begin(); i != r.end(); ++i)
		{
			for (size_t j = 0; j < m_influenceObject.size(); j++)
			{

				MVector toolCentre = m_toolCentre[j];
				MPoint radius = m_infuenceRadius[j];
				int objType = m_influenceObject[j];
						
				MVector pointOnMesh = MPoint(m_meshPoints[i]) * m_localToWorldMatrix;
				MVector directionVector(pointOnMesh - toolCentre);
				double p = directionVector.length();

				MPoint pointOnInflObj;

				if (objType == InfluenceType::kSphere)
				{
					// A sphere with a user defined constant radius
					pointOnInflObj = MPoint(radius.x, radius.x, radius.x,1.0f);
				}
				else if (objType == InfluenceType::kEllipsoid)
				{
					// Parametrization of ellipse with variable values for x y and z radii
					pointOnInflObj = MPoint(radius.x, radius.y, radius.z, 1.0f);
				}

				MPoint boundryIntersectionPoint = toolCentre + pointOnInflObj;
				double p0 = MVector(boundryIntersectionPoint - toolCentre).length();
				directionVector.normalize();

				double p_prime = std::cbrt(std::pow(p0, 3) + std::pow(p, 3));
				MPoint M_prime = toolCentre + p_prime * directionVector;
				pointOnMesh = M_prime * m_worldToLocalMatrix;
				
				m_meshPoints.set(pointOnMesh, i);
			}
		}
		
	}

	const std::vector<unsigned int> m_influenceObject;
	const std::vector <MPoint> m_infuenceRadius;
	const std::vector <MVector> m_toolCentre;
	const MMatrix m_worldToLocalMatrix;
	const MMatrix m_localToWorldMatrix;
	MFloatPointArray& m_meshPoints;
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

	// Getting the mesh from the output Geom plug
	MObject thisNode = this->thisMObject();
	MPlug outPlug(thisNode, outputGeom);
	status = outPlug.selectAncestorLogicalIndex(0, outputGeom);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MDataHandle outputData = data.outputValue(outPlug);
	
	if (outputData.type() != MFnData::kMesh) 
	{
		std::cerr << "Incorrect output mesh type" << std::endl;
		return MStatus::kFailure;
	}
	MObject oSurf = outputData.asMesh();
	if (oSurf.isNull()) 
	{
		std::cout << "Output surface is NULL" << std::endl ;
		return MStatus::kFailure;
	}

	// Getting all points of the mesh
	MFnMesh outMesh;
	outMesh.setObject(oSurf);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFloatPointArray origPoints;
	outMesh.getPoints(origPoints, MSpace::kTransform);

	// Getting all the input arrays for radius, object type and position
	MArrayDataHandle inputArray = data.inputArrayValue(a_compoundInfluences);
	unsigned int inputArrayCount = inputArray.elementCount();
	MArrayDataHandle radiusArray = data.inputArrayValue(a_compoundRadius);
	unsigned int radiusArrayCount = radiusArray.elementCount();
	MArrayDataHandle objTypeArray = data.inputArrayValue(a_objectType);
	unsigned int objTypeCount = objTypeArray.elementCount();

	if ((inputArrayCount != radiusArrayCount) || (inputArrayCount != objTypeCount))
	{
		// Returning an error if the indices dont match
		MGlobal::displayError("Count does not match between influence Object , their radii and their type");
		return MStatus::kFailure;
	}
	
	std::vector<MVector> H_vectors(inputArrayCount);
	std::vector<MPoint> radiusVec(radiusArrayCount);
	std::vector<unsigned int> objectType(objTypeCount);
	
	for (unsigned int i = 0; i < inputArrayCount; i++)
	{
		// Populating the values of each of the compound attributes in an array
		H_vectors[i] = inputArray.inputValue().child(a_inflPoint).asFloat3();
		radiusVec[i] = radiusArray.inputValue().child(a_inflRadii).asFloat3();
		objectType[i] = objTypeArray.inputValue().asInt();
			
		objTypeArray.next();
		inputArray.next();
		radiusArray.next();
	}

	tbb::task_scheduler_init init;
	// Executing the deformation as a tbb parallel for 
	tbb::parallel_for(
			tbb::blocked_range<size_t>(0, origPoints.length()),
		parallel_deform(
				objectType,
				radiusVec,
				H_vectors,
				worldToLocalMatrix,
				localToWorldMatrix,
				origPoints
			)
	);

	outMesh.setPoints(origPoints, MSpace::kTransform);

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