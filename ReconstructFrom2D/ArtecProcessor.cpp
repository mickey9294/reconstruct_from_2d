#include "ArtecProcessor.h"

#define ENABLE_TEXTURE_MAPPING

// Saving the results takes time and needs considerable amount
// of disc space. Uncomment one or both of the following macros
// in order to enable it. Make sure you have a subdirectory
// designated as OUTPUT_DIR in the current directory
// for output files to be placed.
#define OUTPUT_DIR L"scans"
//#define SAVE_FUSION_MESH_ON
//#define SAVE_TEXTURED_MESH_ON

// simple error log handling for SDK calls
#define SDK_STRINGIFY(x) #x
#define SDK_STRING(x) SDK_STRINGIFY(x)
#define SAFE_SDK_CALL(x)                                                 \
{                                                                        \
	asdk::ErrorCode ec = (x);                                            \
if (ec != asdk::ErrorCode_OK)                                      \
	{                                                                    \
	reportError(ec, __FILE__ " [ line " SDK_STRING(__LINE__) " ]"); \
	return ec;                                                       \
	}                                                                    \
}

void reportError(asdk::ErrorCode ec, const char *place)
{
	const wchar_t* msg = L"No error";

	switch (ec) {

	case asdk::ErrorCode_OutOfMemory:
		msg = L"Not enough storage is available to process the operation";
		break;

	case asdk::ErrorCode_ArgumentInvalid:
		msg = L"Provided argument is invalid";
		break;

	case asdk::ErrorCode_OperationInvalid:
		msg = L"Requested operation is invalid";
		break;

	case asdk::ErrorCode_FormatUnsupported:
		msg = L"Data format is unsupported or invalid";
		break;

	case asdk::ErrorCode_ScannerNotConnected:
		msg = L"Requested scanner is not connected";
		break;

	case asdk::ErrorCode_ScannerNotLicensed:
		msg = L"Requested scanner is not licensed";
		break;

	case asdk::ErrorCode_ScannerLocked:
		msg = L"Requested scanner is already used by someone else";
		break;

	case asdk::ErrorCode_ScannerInitializationFailed:
		msg = L"Scanner initialization failed";
		break;

	case asdk::ErrorCode_FrameCorrupted:
		msg = L"Frame is corrupted";
		break;

	case asdk::ErrorCode_FrameReconstructionFailed:
		msg = L"Frame reconstruction failed";
		break;

	case asdk::ErrorCode_FrameRegistrationFailed:
		msg = L"Frame registration failed";
		break;

	case asdk::ErrorCode_OperationUnsupported:
		msg = L"Requested operation is unsupported. Check versions";
		break;

	case asdk::ErrorCode_OperationDenied:
		msg = L"Requested operation is denied. Check your license(s)";
		break;

	case asdk::ErrorCode_OperationFailed:
		msg = L"Requested operation has failed";
		break;

	case asdk::ErrorCode_OperationAborted:
		msg = L"Requested operation was canceled from client's side";
		break;

	default:
		msg = L"Unexplained error";
		break;
	}

	std::wcerr << msg << " [error " << std::hex << ec << "] " << "at " << place;
}

ArtecProcessor::ArtecProcessor()
{
}


ArtecProcessor::~ArtecProcessor()
{
}

asdk::ErrorCode ArtecProcessor::set_input_cloud(std::shared_ptr<ShapeModel> model)
{
	asdk::TRef<asdk::IModel> inputContainer;
	asdk::TRef<asdk::IModel> outputContainer;
	asdk::TRef<asdk::ICancellationTokenSource> ctSource;
	SAFE_SDK_CALL(asdk::createModel(&inputContainer));
	SAFE_SDK_CALL(asdk::createModel(&outputContainer));
	SAFE_SDK_CALL(asdk::createCancellationTokenSource(&ctSource));

	asdk::TRef<asdk::IFrameMesh> mesh;
	SAFE_SDK_CALL(asdk::createFrameMesh(&mesh, model->num_vertices()));

	asdk::TRef<asdk::IArrayPoint3F> points_array;
	SAFE_SDK_CALL(asdk::createArrayPoint3F(&points_array, model->num_vertices()));

	asdk::Point3F *points = points_array->getPointer();
	for (int i = 0; i < model->num_vertices(); i++)
	{
		Eigen::Vector3d &vert = model->operator[](i);
		points[i].x = vert.x();
		points[i].y = vert.y();
		points[i].z = vert.z();
	}

	mesh->setPoints(points_array);

	asdk::TRef<asdk::IScan> iscan;
	SAFE_SDK_CALL(asdk::createScan(&iscan));
	iscan->add(mesh);

	inputContainer->add(iscan);

	workset_ = { inputContainer, outputContainer, 0, ctSource->getToken(), 0 };

	return asdk::ErrorCode_OK;
}

asdk::ErrorCode ArtecProcessor::set_input_cloud(std::vector<std::shared_ptr<ShapeModel>>& frames)
{
	asdk::TRef<asdk::IModel> inputContainer;
	asdk::TRef<asdk::IModel> outputContainer;
	asdk::TRef<asdk::ICancellationTokenSource> ctSource;
	SAFE_SDK_CALL(asdk::createModel(&inputContainer));
	SAFE_SDK_CALL(asdk::createModel(&outputContainer));
	SAFE_SDK_CALL(asdk::createCancellationTokenSource(&ctSource));

	asdk::TRef<asdk::IScan> iscan;
	SAFE_SDK_CALL(asdk::createScan(&iscan));

	for (std::vector<std::shared_ptr<ShapeModel>>::iterator it = frames.begin(); it != frames.end(); ++it)
	{
		asdk::TRef<asdk::IFrameMesh> mesh;
		SAFE_SDK_CALL(asdk::createFrameMesh(&mesh, (*it)->num_vertices()));

		asdk::TRef<asdk::IArrayPoint3F> points_array;
		SAFE_SDK_CALL(asdk::createArrayPoint3F(&points_array, (*it)->num_vertices()));
		asdk::Point3F *points = points_array->getPointer();
		for (int i = 0; i < (*it)->num_vertices(); i++)
		{
			Eigen::Vector3d &vert = (*it)->operator[](i);
			points[i].x = vert.x();
			points[i].y = vert.y();
			points[i].z = vert.z();
		}

		mesh->setPoints(points_array);
		
		iscan->add(mesh);
	}

	inputContainer->add(iscan);

	workset_ = { inputContainer, outputContainer, 0, ctSource->getToken(), 0 };

	return asdk::ErrorCode_OK;
}

asdk::ErrorCode ArtecProcessor::pointcloud_register()
{
	asdk::ScannerType scannerType = asdk::ScannerType_Eva;

	// apply serial registration
	//{
	//	qDebug() << "Creating serial registration procedure...";

	//	TRef<asdk::IAlgorithm> serialRegistration;
	//	asdk::SerialRegistrationSettings serialDesc = {
	//		scannerType, asdk::SerialRegistrationType_FineTextured
	//	};

	//	asdk::ErrorCode ec = asdk::createSerialRegistrationAlgorithm(&serialRegistration, &serialDesc);
	//	qDebug() << "ok";
	//	asdk::ErrorCodeope

	//	qDebug() << "Launching the serial registration algorithm...";
	//	SAFE_SDK_CALL(asdk::executeJob(serialRegistration, &workset_));
	//	qDebug() << "ok";
	//}

	//// prepare serial registration output for the global registration
	//std::swap(workset_.in, workset_.out);
	//workset_.out->clear();

	//// proceed with global registration
	//{
	//	qDebug() << "Creating global registration procedure...";

	//	TRef<asdk::IAlgorithm> globalRegistration;
	//	asdk::GlobalRegistrationSettings globalDesc = {
	//		scannerType, asdk::GlobalRegistrationType_Geometry
	//	};
	//	SAFE_SDK_CALL(asdk::createGlobalRegistrationAlgorithm(&globalRegistration, &globalDesc));
	//	qDebug() << "ok";


	//	qDebug() << "Launching the global registration algorithm...";
	//	SAFE_SDK_CALL(asdk::executeJob(globalRegistration, &workset_));
	//	qDebug() << "ok";
	//}

	//// prepare global registration output for outliers removal
	//std::swap(workset_.in, workset_.out);
	//workset_.out->clear();


	// apply outliers removal
	{
		qDebug() << "Creating outliers removal procedure...";

		TRef<asdk::IAlgorithm> noOutliers;
		asdk::OutliersRemovalSettings outliersDesc;
		// get default settings
		SAFE_SDK_CALL(asdk::initializeOutliersRemovalSettings(&outliersDesc, scannerType));
		SAFE_SDK_CALL(asdk::createOutliersRemovalAlgorithm(&noOutliers, &outliersDesc));
		qDebug() << "ok";


		qDebug() << "Launching the outliers removal algorithm...";
		SAFE_SDK_CALL(asdk::executeJob(noOutliers, &workset_));
		qDebug() << "ok";
	}

	save_refined_pointcloud(workset_.out);

	// prepare global registration output for outliers removal
	std::swap(workset_.in, workset_.out);
	workset_.out->clear();

	return asdk::ErrorCode_OK;
}

void ArtecProcessor::save_refined_pointcloud(asdk::IModel * imodel)
{
	asdk::IScan *iscan = imodel->getElement(0);

	std::shared_ptr<PCModel> pc(new PCModel());

	for (int i = 0; i < iscan->getSize(); i++)
	{
		asdk::IFrameMesh *frame = iscan->getElement(i);

		asdk::Matrix4x4D trans_mat = iscan->getTransformation(i);

		Eigen::Matrix4d q_trans_mat;
		for (int row = 0; row < 4; row++)
		{
			for (int col = 0; col < 4; col++)
			{
				q_trans_mat(row, col) = trans_mat(row, col);
			}
		}

		pc->add_frame(frame, q_trans_mat);
	}

	/* Write point cloud */
	std::string off_path = "..\\data\\output\\out.off";
	//std::string pts_path = "D:\\Projects\\shape2pose\\data\\2_analysis\\coseg_chairs\\points\\test.pts";
	pc->output(off_path.c_str());
	//pc->output(pts_path.c_str());
}