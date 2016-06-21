#include "pointanalysis.h"

PointAnalysis::PointAnalysis(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	qRegisterMetaType<PCModel *>("PCModelPointer");
	qRegisterMetaType<PAPointCloud *>("PAPointCloud");
	qRegisterMetaType<QVector<int>>("QVectorInt");
	qRegisterMetaType<QVector<OBB *>>("OBBPointer");
	qRegisterMetaType<QVector<PAPart>>("PAPartVector");
	
	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(load()));
	connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(saveModel()));
	connect(ui.actionEstimate_Features, SIGNAL(triggered()), this, SLOT(estimateFeatures()));
	connect(ui.actionExtract_Point_Features, SIGNAL(triggered()), this, SLOT(extractPointFeatures()));
	connect(&pfe, SIGNAL(reportStatus(QString)), this, SLOT(setStatMessage(QString)));
	connect(&pfe, SIGNAL(showModel(PCModel *)), ui.displayGLWidget, SLOT(setModel(PCModel *)));
	connect(&pfe, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(ui.actionTrain_Point_Classifier, SIGNAL(triggered()), this, SLOT(trainPointClassifier()));
	connect(ui.actionTest_PointCloud, SIGNAL(triggered()), this, SLOT(testPointCloud()));
	connect(ui.displayGLWidget, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(&testPcThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(&testPcThread, SIGNAL(reportStatus(QString)), this, SLOT(setStatMessage(QString)));
	connect(ui.actionCompute_sdf, SIGNAL(triggered()), this, SLOT(computeSdf()));
	connect(ui.actionNormalize_Meshes, SIGNAL(triggered()), this, SLOT(normalizeMeshes()));
	connect(ui.actionCompute_OBB, SIGNAL(triggered()), this, SLOT(computeOBB()));
	connect(ui.actionTrain_Parts_Relations, SIGNAL(triggered()), this, SLOT(trainPartRelations()));
	connect(ui.actionStructure_Inference, SIGNAL(triggered()), this, SLOT(inferStructure()));
	connect(&m_analyser, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));

	fe = NULL;
	trainThread = NULL;
	loadThread = NULL;
	outputThread = NULL;
	pcaThread = NULL;
	trainPartsThread = NULL;
	normalizeThread = NULL;
	sdfThread = NULL;
	trainThread = NULL;
	
	m_modelClassName = "coseg_chairs_3";
}

PointAnalysis::~PointAnalysis()
{

}

void PointAnalysis::load()
{
	if (loadThread != NULL)
	{
		if (loadThread->isRunning())
			loadThread->terminate();
		delete(loadThread);
		loadThread = NULL;
	}

	/* User choose a model file by FileDialog */
	QString filepath = QFileDialog::getOpenFileName(this, tr("Load"), 
		"../../Data", 
		tr("Object File Format (*.off);;XYZ Point Cloud (*.xyz)"));

	if (filepath.length() > 0){    /* If the user choose a valid model file path */
		filename = Utils::getModelName(filepath).toStdString();
		/* Set the status bar to inform users it is loading now */
		QString stat_msg = "Loading point cloud from " + filepath + "...";
		onDebugTextAdded(stat_msg);
		ui.statusBar->showMessage(stat_msg, 0);

		/* Start LoadThread to load the model */
		loadThread = new LoadThread(filepath.toStdString(), LoadThread::PHASE::TESTING, this);
		connect(loadThread, SIGNAL(loadPointsCompleted(PCModel *)), this, SLOT(loadCompleted(PCModel *)));
		loadThread->start();
	}
}

void PointAnalysis::loadCompleted(PCModel *model)
{
	ui.displayGLWidget->setModel(model);
	connect(ui.displayGLWidget->getModel(), SIGNAL(addDebugText(QString)), ui.displayGLWidget, SLOT(onDebugTextAdded(QString)));
	connect(ui.displayGLWidget->getModel(), SIGNAL(onLabelsChanged()), ui.displayGLWidget, SLOT(updateLabels()));
	ui.statusBar->showMessage("Loading done.");
	onDebugTextAdded("Loading done.");

	if (loadThread != NULL && loadThread->isRunning())
		loadThread->terminate();
	delete(loadThread);
	loadThread = NULL;
}

void PointAnalysis::saveModel()
{
	if (outputThread != NULL)
	{
		if (outputThread->isRunning())
			outputThread->terminate();
		delete(outputThread);
		outputThread = NULL;
	}

	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
		"../../Data",
		tr("Object File Format (*.off);;Stanford Polygon File Format (*.ply"));
	
	connect(ui.displayGLWidget->getModel(), SIGNAL(outputProgressReport(int)), &progressDialog1, SLOT(setProgressValue(int)));
	outputThread = new OutputThread(this);
	connect(outputThread, SIGNAL(outputCompleted()), this, SLOT(saveCompleted()));
	outputThread->setOutputModel(ui.displayGLWidget->getModel(), fileName.toStdString());
	outputThread->start();
	progressDialog1.setProgressValue(0);
	progressDialog1.setWindowTitle("Point Cloud Save");
	progressDialog1.setMessage("Saving the point cloud, please wait for a few seconds...");
	progressDialog1.exec();
}

void PointAnalysis::saveCompleted()
{
	progressDialog1.done(0);

	if (outputThread != NULL)
	{
		if (outputThread->isRunning())
			outputThread->terminate();
		delete(outputThread);
		outputThread = NULL;
	}
}

void PointAnalysis::getProgressReport(int value)
{
	ui.mainProgressBar->setValue(value);
}

void PointAnalysis::estimateFeatures()
{
	if (fe != NULL)
	{
		delete(fe);
		fe = NULL;
	}
	qDebug() << "Estimate point features.";
	onDebugTextAdded("Estimate point features of " + QString::fromStdString(filename) + ".");
	ui.statusBar->showMessage("Estimating points features of " + QString::fromStdString(filename) + "...");
	fe = new FeatureEstimator(ui.displayGLWidget->getModel(), FeatureEstimator::PHASE::TESTING, this);
	connect(fe, SIGNAL(estimateCompleted(PAPointCloud *)), this, SLOT(featureEstimateCompleted(PAPointCloud *)));
	connect(fe, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	fe->estimateFeatures();
}

void PointAnalysis::featureEstimateCompleted(PAPointCloud *pointcloud)
{
	qDebug() << "Point features estimation done.";
	onDebugTextAdded("Point features estimation done.");
	std::string outputname = "../data/features_test/" + filename + ".csv";
	onDebugTextAdded("Save the point cloud features to file " + QString::fromStdString(outputname) + ".");
	setStatMessage("Saving the point cloud features to file" + QString::fromStdString(outputname) + "...");
	pointcloud->writeToFile(outputname.c_str());
	disconnect(fe, SIGNAL(estimateCompleted(PAPointCloud *)), this, SLOT(featureEstimateCompleted(PAPointCloud *)));
	ui.statusBar->showMessage("Point features estimation done.");
}

void PointAnalysis::setStatMessage(QString msg)
{
	ui.statusBar->showMessage(msg, 0);
}

void PointAnalysis::extractPointFeatures()
{
	pfe.execute();
}

void PointAnalysis::trainPointClassifier()
{
	if (trainThread != NULL)
	{
		if (trainThread->isRunning())
			trainThread->terminate();
		delete(trainThread);
		trainThread = NULL;
	}

	trainThread = new TrainThread(m_modelClassName, this);
	connect(trainThread, SIGNAL(finished()), this, SLOT(onTrainingCompleted()));
	connect(trainThread, SIGNAL(reportStatus(QString)), this, SLOT(setStatMessage(QString)));
	connect(trainThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	trainThread->start();
}

void PointAnalysis::onTrainingCompleted()
{
	if (trainThread != NULL)
	{
		if (trainThread->isRunning())
			trainThread->terminate();
		delete(trainThread);
		trainThread = NULL;
	}
}

void PointAnalysis::onDebugTextAdded(QString text)
{
	ui.debugTextEdit->append(text);
}

void PointAnalysis::testPointCloud()
{
	if (testPcThread.isRunning())
		testPcThread.terminate();

	connect(&testPcThread, SIGNAL(setPCLabels(QVector<int>)), this, SLOT(onTestCompleted(QVector<int>)));
	connect(&testPcThread, SIGNAL(signalTest()), ui.displayGLWidget->getModel(), SLOT(receiveSignalTest()));
	testPcThread.setPcName(QString::fromStdString(filename));
	testPcThread.start();
}

void PointAnalysis::onTestCompleted(QVector<int> labels)
{
	onDebugTextAdded("Test Completed. Set the labels to point cloud model.");
	ui.displayGLWidget->getModel()->setLabels(labels);
}

void PointAnalysis::computeSdf()
{
	sdfThread = new SdfThread(m_modelClassName, this);
	connect(sdfThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(sdfThread, SIGNAL(computeSdfCompleted()), this, SLOT(onComputeSdfDone()));
	sdfThread->execute();
}

void PointAnalysis::onComputeSdfDone()
{
	onDebugTextAdded("Sdf computation done.");
}

void PointAnalysis::normalizeMeshes()
{
	if (normalizeThread != NULL)
	{
		if (normalizeThread->isRunning())
			normalizeThread->terminate();
		delete(normalizeThread);
		normalizeThread = NULL;
	}

	QString file_list_path = QFileDialog::getOpenFileName(this, tr("Choose the model list file"),
		"../data",
		tr("Model List File (*.txt)"));

	normalizeThread = new NormalizeThread(m_modelClassName, this);
	connect(normalizeThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(normalizeThread, SIGNAL(finished()), this, SLOT(onNormalizeDone()));
	normalizeThread->start();
}

void PointAnalysis::onNormalizeDone()
{
	if (normalizeThread != NULL)
	{
		if (normalizeThread->isRunning())
			normalizeThread->terminate();
		delete(normalizeThread);
		normalizeThread = NULL;
	}
}

void PointAnalysis::computeOBB()
{
	if (pcaThread != NULL)
	{
		if (pcaThread->isRunning())
			pcaThread->terminate();
		delete(pcaThread);
		pcaThread = NULL;
	}

	pcaThread = new PCAThread(ui.displayGLWidget->getModel());
	connect(pcaThread, SIGNAL(finished()), this, SLOT(onComputeOBBDone()));
	connect(pcaThread, SIGNAL(estimateOBBsCompleted(QVector<OBB *>)), ui.displayGLWidget, SLOT(setOBBs(QVector<OBB *>)));
	connect(pcaThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	pcaThread->start();
}

void PointAnalysis::onComputeOBBDone()
{
	if (pcaThread != NULL)
	{
		if (pcaThread->isRunning())
			pcaThread->terminate();
		delete(pcaThread);
		pcaThread = NULL;
	}
}

void PointAnalysis::trainPartRelations()
{
	if (trainPartsThread != NULL)
	{
		if (trainPartsThread->isRunning())
			trainPartsThread->terminate();
		delete(trainPartsThread);
		trainPartsThread = NULL;
	}

	trainPartsThread = new TrainPartsThread(this);
	connect(trainPartsThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(trainPartsThread, SIGNAL(showModel(PCModel *)), ui.displayGLWidget, SLOT(setModel(PCModel *)));
	connect(&trainPartsThread->pcaThread, SIGNAL(estimateOBBsCompleted(QVector<OBB*>)), ui.displayGLWidget, SLOT(setOBBs(QVector<OBB *>)));
	connect(trainPartsThread, SIGNAL(finish()), this, SLOT(onTrainPartsDone()));
	/* No need "trainPartsThread->start();", the thread will start itself after construction */
}

void PointAnalysis::onTrainPartsDone()
{
	if (trainPartsThread != NULL)
	{
		if (trainPartsThread->isRunning())
			trainPartsThread->terminate();
		delete(trainPartsThread);
		trainPartsThread = NULL;
	}
}

void PointAnalysis::inferStructure()
{
	m_analyser.setPointCloud(ui.displayGLWidget->getModel());
	m_analyser.execute();
}