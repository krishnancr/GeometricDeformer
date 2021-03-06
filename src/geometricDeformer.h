#ifndef SPHERECOLLIDEDEFORMER_H
#define SPHERECOLLIDEDEFORMER_H

#include <maya/MDagModifier.h>
#include <maya/MGlobal.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <maya/MPxDeformerNode.h>

class GeometricDeformer : public MPxDeformerNode
{
public:
	GeometricDeformer();
	virtual				~GeometricDeformer();
	static  void*		creator();

	virtual MStatus     deform(MDataBlock& data,
		MItGeometry& itGeo,
		const MMatrix& localToWorldMatrix,
		unsigned int geomIndex);


	static  MStatus		initialize();

	static MObject a_compoundInfluences;
	static MObject a_inflPoint;
	static MObject a_compoundRadius;
	static MObject a_inflRadii; 
	static MObject a_objectType;

	


	static MTypeId	id;

};
#endif
