#include <maya/MFnPlugin.h>
#include "geometricDeformer.h"

MStatus initializePlugin(MObject obj)
{
	MStatus status;

	MFnPlugin fnPlugin(obj, "Simple Plugin", "1.0", "Any");

	status = fnPlugin.registerNode("GeometricDeformer",
		GeometricDeformer::id,
		GeometricDeformer::creator,
		GeometricDeformer::initialize,
		MPxNode::kDeformerNode);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}


MStatus uninitializePlugin(MObject obj)
{
	MStatus status;

	MFnPlugin fnPlugin(obj);

	status = fnPlugin.deregisterNode(GeometricDeformer::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}