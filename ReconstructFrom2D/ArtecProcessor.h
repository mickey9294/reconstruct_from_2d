#ifndef ARTECPROCESSOR_H
#define ARTECPROCESSOR_H

#include <QDebug>

#include <artec/sdk/base/Log.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/RefBase.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/ICompositeContainer.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/ICancellationTokenSource.h>
#include <artec/sdk/base/IO/ObjIO.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IScannerObserver.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/scanning/IScanningProcedure.h>
#include <artec/sdk/scanning/IArrayScanner.h>
#include <artec/sdk/scanning/IScanningProcedureBundle.h>
#include <artec/sdk/algorithms/IAlgorithm.h>
#include <artec/sdk/algorithms/Algorithms.h>

#include "PCModel.h"

namespace asdk {
	using namespace artec::sdk::base;
	using namespace artec::sdk::capturing;
	using namespace artec::sdk::scanning;
	using namespace artec::sdk::algorithms;
};

using asdk::TRef;

class ArtecProcessor
{
public:
	ArtecProcessor();
	~ArtecProcessor();

	asdk::ErrorCode set_input_cloud(std::shared_ptr<ShapeModel> model);
	asdk::ErrorCode set_input_cloud(std::vector<std::shared_ptr<ShapeModel>> &frames);
	asdk::ErrorCode pointcloud_register();

private:
	asdk::AlgorithmWorkset workset_;

	void save_refined_pointcloud(asdk::IModel * imodel);
};

#endif