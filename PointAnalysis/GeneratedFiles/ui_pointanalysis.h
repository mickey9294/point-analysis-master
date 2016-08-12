/********************************************************************************
** Form generated from reading UI file 'pointanalysis.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POINTANALYSIS_H
#define UI_POINTANALYSIS_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include <myglwidget.h>

QT_BEGIN_NAMESPACE

class Ui_PointAnalysisClass
{
public:
    QAction *actionOpen;
    QAction *actionSave;
    QAction *actionSave_As;
    QAction *actionExit;
    QAction *actionAbout;
    QAction *actionEstimate_Features;
    QAction *actionExtract_Point_Features;
    QAction *actionTrain_Point_Classifier;
    QAction *actionTest_Data;
    QAction *actionTest_PointCloud;
    QAction *actionCompute_sdf;
    QAction *actionNormalize_Meshes;
    QAction *actionCompute_OBB;
    QAction *actionTrain_Parts_Relations;
    QAction *actionStructure_Inference;
    QAction *actionDebug_Parts_Relations;
    QAction *actionCheck_Models;
    QWidget *centralWidget;
    MyGLWidget *displayGLWidget;
    QProgressBar *mainProgressBar;
    QLabel *mainProgressMessage;
    QTextEdit *debugTextEdit;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuHelp;
    QMenu *menuFeature;
    QMenu *menuTest;
    QMenu *menuInference;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PointAnalysisClass)
    {
        if (PointAnalysisClass->objectName().isEmpty())
            PointAnalysisClass->setObjectName(QStringLiteral("PointAnalysisClass"));
        PointAnalysisClass->resize(848, 820);
        actionOpen = new QAction(PointAnalysisClass);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionSave = new QAction(PointAnalysisClass);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        actionSave_As = new QAction(PointAnalysisClass);
        actionSave_As->setObjectName(QStringLiteral("actionSave_As"));
        actionExit = new QAction(PointAnalysisClass);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionAbout = new QAction(PointAnalysisClass);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionEstimate_Features = new QAction(PointAnalysisClass);
        actionEstimate_Features->setObjectName(QStringLiteral("actionEstimate_Features"));
        actionExtract_Point_Features = new QAction(PointAnalysisClass);
        actionExtract_Point_Features->setObjectName(QStringLiteral("actionExtract_Point_Features"));
        actionTrain_Point_Classifier = new QAction(PointAnalysisClass);
        actionTrain_Point_Classifier->setObjectName(QStringLiteral("actionTrain_Point_Classifier"));
        actionTest_Data = new QAction(PointAnalysisClass);
        actionTest_Data->setObjectName(QStringLiteral("actionTest_Data"));
        actionTest_PointCloud = new QAction(PointAnalysisClass);
        actionTest_PointCloud->setObjectName(QStringLiteral("actionTest_PointCloud"));
        actionCompute_sdf = new QAction(PointAnalysisClass);
        actionCompute_sdf->setObjectName(QStringLiteral("actionCompute_sdf"));
        actionNormalize_Meshes = new QAction(PointAnalysisClass);
        actionNormalize_Meshes->setObjectName(QStringLiteral("actionNormalize_Meshes"));
        actionCompute_OBB = new QAction(PointAnalysisClass);
        actionCompute_OBB->setObjectName(QStringLiteral("actionCompute_OBB"));
        actionTrain_Parts_Relations = new QAction(PointAnalysisClass);
        actionTrain_Parts_Relations->setObjectName(QStringLiteral("actionTrain_Parts_Relations"));
        actionStructure_Inference = new QAction(PointAnalysisClass);
        actionStructure_Inference->setObjectName(QStringLiteral("actionStructure_Inference"));
        actionDebug_Parts_Relations = new QAction(PointAnalysisClass);
        actionDebug_Parts_Relations->setObjectName(QStringLiteral("actionDebug_Parts_Relations"));
        actionCheck_Models = new QAction(PointAnalysisClass);
        actionCheck_Models->setObjectName(QStringLiteral("actionCheck_Models"));
        centralWidget = new QWidget(PointAnalysisClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        displayGLWidget = new MyGLWidget(centralWidget);
        displayGLWidget->setObjectName(QStringLiteral("displayGLWidget"));
        displayGLWidget->setGeometry(QRect(10, 10, 821, 551));
        mainProgressBar = new QProgressBar(centralWidget);
        mainProgressBar->setObjectName(QStringLiteral("mainProgressBar"));
        mainProgressBar->setGeometry(QRect(680, 730, 161, 20));
        mainProgressBar->setValue(0);
        mainProgressBar->setTextVisible(false);
        mainProgressMessage = new QLabel(centralWidget);
        mainProgressMessage->setObjectName(QStringLiteral("mainProgressMessage"));
        mainProgressMessage->setGeometry(QRect(30, 580, 511, 21));
        mainProgressMessage->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        debugTextEdit = new QTextEdit(centralWidget);
        debugTextEdit->setObjectName(QStringLiteral("debugTextEdit"));
        debugTextEdit->setGeometry(QRect(10, 570, 661, 181));
        debugTextEdit->setReadOnly(true);
        PointAnalysisClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(PointAnalysisClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 848, 23));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        menuFeature = new QMenu(menuBar);
        menuFeature->setObjectName(QStringLiteral("menuFeature"));
        menuTest = new QMenu(menuBar);
        menuTest->setObjectName(QStringLiteral("menuTest"));
        menuInference = new QMenu(menuBar);
        menuInference->setObjectName(QStringLiteral("menuInference"));
        PointAnalysisClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(PointAnalysisClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        PointAnalysisClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(PointAnalysisClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        PointAnalysisClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuFeature->menuAction());
        menuBar->addAction(menuTest->menuAction());
        menuBar->addAction(menuInference->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionSave_As);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuHelp->addAction(actionAbout);
        menuFeature->addAction(actionEstimate_Features);
        menuFeature->addAction(actionExtract_Point_Features);
        menuFeature->addAction(actionTrain_Point_Classifier);
        menuFeature->addAction(actionCompute_sdf);
        menuFeature->addAction(actionNormalize_Meshes);
        menuFeature->addAction(actionCompute_OBB);
        menuFeature->addAction(actionTrain_Parts_Relations);
        menuFeature->addAction(actionDebug_Parts_Relations);
        menuFeature->addAction(actionCheck_Models);
        menuTest->addAction(actionTest_Data);
        menuTest->addAction(actionTest_PointCloud);
        menuInference->addAction(actionStructure_Inference);

        retranslateUi(PointAnalysisClass);

        QMetaObject::connectSlotsByName(PointAnalysisClass);
    } // setupUi

    void retranslateUi(QMainWindow *PointAnalysisClass)
    {
        PointAnalysisClass->setWindowTitle(QApplication::translate("PointAnalysisClass", "PointAnalysis", 0));
        actionOpen->setText(QApplication::translate("PointAnalysisClass", "Open", 0));
        actionSave->setText(QApplication::translate("PointAnalysisClass", "Save", 0));
        actionSave_As->setText(QApplication::translate("PointAnalysisClass", "Save As", 0));
        actionExit->setText(QApplication::translate("PointAnalysisClass", "Exit", 0));
        actionAbout->setText(QApplication::translate("PointAnalysisClass", "About", 0));
        actionEstimate_Features->setText(QApplication::translate("PointAnalysisClass", "Estimate Features", 0));
        actionExtract_Point_Features->setText(QApplication::translate("PointAnalysisClass", "Extract Point Features", 0));
        actionTrain_Point_Classifier->setText(QApplication::translate("PointAnalysisClass", "Train Point Classifier", 0));
        actionTest_Data->setText(QApplication::translate("PointAnalysisClass", "Test Data", 0));
        actionTest_PointCloud->setText(QApplication::translate("PointAnalysisClass", "Test Point Cloud", 0));
        actionCompute_sdf->setText(QApplication::translate("PointAnalysisClass", "Compute sdf", 0));
        actionNormalize_Meshes->setText(QApplication::translate("PointAnalysisClass", "Normalize Meshes", 0));
        actionCompute_OBB->setText(QApplication::translate("PointAnalysisClass", "Compute OBB", 0));
        actionTrain_Parts_Relations->setText(QApplication::translate("PointAnalysisClass", "Train Parts Relations", 0));
        actionStructure_Inference->setText(QApplication::translate("PointAnalysisClass", "Structure Inference", 0));
        actionDebug_Parts_Relations->setText(QApplication::translate("PointAnalysisClass", "Debug Parts Relations", 0));
        actionCheck_Models->setText(QApplication::translate("PointAnalysisClass", "Check Models", 0));
        mainProgressMessage->setText(QApplication::translate("PointAnalysisClass", "<html><head/><body><p align=\"right\"><br/></p></body></html>", 0));
        menuFile->setTitle(QApplication::translate("PointAnalysisClass", "File", 0));
        menuHelp->setTitle(QApplication::translate("PointAnalysisClass", "Help", 0));
        menuFeature->setTitle(QApplication::translate("PointAnalysisClass", "Train", 0));
        menuTest->setTitle(QApplication::translate("PointAnalysisClass", "Test", 0));
        menuInference->setTitle(QApplication::translate("PointAnalysisClass", "Inference", 0));
    } // retranslateUi

};

namespace Ui {
    class PointAnalysisClass: public Ui_PointAnalysisClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINTANALYSIS_H
