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

				MVector H = m_toolCentre[j];
				MPoint radius = m_infuenceRadius[j];
				int objType = m_influenceObject[j];
						
				MPoint point = m_meshPoints[i];
				MVector M = point * m_localToWorldMatrix;
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
					double theta = 1.507;
					double gamma = 1.507;
					point_on_inflObj.x = radius.x * std::sin(gamma) * std::cos(theta);
					point_on_inflObj.y = radius.y * std::sin(gamma) * std::sin(theta);
					point_on_inflObj.z = radius.z * std::cos(gamma);
				}

				MPoint I = H + point_on_inflObj;
				double p0 = MVector(I - H).length();
				u.normalize();
				double p_prime = std::cbrt(std::pow(p0, 3) + std::pow(p, 3)) ;
				MPoint M_prime = H + p_prime * u;
				M_prime *= m_worldToLocalMatrix;
				
				m_meshPoints.set(M_prime, i);
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
	
	/*
	
		Stuff setup for tbb parallelization 
	*/
	
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

	MFnMesh outMesh;
	outMesh.setObject(oSurf);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFloatPointArray origPoints;
	MFloatPointArray finalPoints(origPoints);
	outMesh.getPoints(origPoints, MSpace::kTransform);

	MArrayDataHandle inputArray = data.inputArrayValue(a_compoundInfluences);
	unsigned int inputArrayCount = inputArray.elementCount();
	MArrayDataHandle radiusArray = data.inputArrayValue(a_compoundRadius);
	unsigned int radiusArrayCount = radiusArray.elementCount();
	MArrayDataHandle objTypeArray = data.inputArrayValue(a_objectType);
	unsigned int objTypeCount = objTypeArray.elementCount();

	if ((inputArrayCount != radiusArrayCount) || (inputArrayCount != objTypeCount))
	{
		MGlobal::displayError("Count does not match between influence Object , their radii and their type");
		return MStatus::kFailure;
	}
	
	std::vector<MVector> H_vectors(inputArrayCount);
	std::vector<MPoint> radiusVec(radiusArrayCount);
	std::vector<unsigned int> objectType(objTypeCount);
	
	for (unsigned int i = 0; i < inputArrayCount; i++)
	{
		H_vectors[i] = inputArray.inputValue().child(a_inflPoint).asFloat3();
		radiusVec[i] = radiusArray.inputValue().child(a_inflRadii).asFloat3();
		objectType[i] = objTypeArray.inputValue().asInt();
			
		objTypeArray.next();
		inputArray.next();
		radiusArray.next();
	}

	tbb::task_scheduler_init init;
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