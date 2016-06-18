#ifndef DISPLAYGLWIDGET_H
#define DISPLAYGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QMatrix4x4>
#include <QWheelEvent>
#include <QMouseEvent>
#include "pcmodel.h"
#include <QtDebug>
#include <QtCore>
#include <queue>
#include <fstream>
#include <string>
#include "obb.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

class DisplayGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	DisplayGLWidget(QWidget *parent);
	~DisplayGLWidget();
	void clearScene();
	PCModel *getModel();

	public slots:
	void cleanup();
	void setModel(PCModel *model);
	void setOBBs(QVector<OBB *> obbs);
	void outputPointCloud();
	void updateLabels();
	void onDebugTextAdded(QString text);

signals:
	void startRecon(int no);
	void startOutput(PCModel *model, int no);
	void addDebugText(QString text);

protected:
	void initializeGL() Q_DECL_OVERRIDE;
	void paintGL() Q_DECL_OVERRIDE;
	void resizeGL(int width, int height) Q_DECL_OVERRIDE;
	void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void wheelEvent(QWheelEvent *e);

private:
	void setupVertexAttribs();
	void setXRotation(int angle);
	void setYRotation(int angle);
	void setZRotation(int angle);

	const double zfar = 0.0;
	const double znear = 5.0;
	bool m_core;
	int m_xRot;
	int m_yRot;
	int m_zRot;
	double m_cameraPositionZ;
	QPoint m_lastPos;
	QOpenGLVertexArrayObject m_vao;
	QOpenGLBuffer m_meshModelVbo;
	QVector<QOpenGLBuffer> m_obbVbos;
	QOpenGLShaderProgram *m_program;
	PCModel *m_model;
	QVector<OBB *> m_OBBs;
	int m_projMatrixLoc;
	int m_mvMatrixLoc;
	int m_normalMatrixLoc;
	int m_lightPosLoc;
	int m_colorLoc;
	QVector3D m_color;
	QMatrix4x4 m_proj;
	QMatrix4x4 m_camera;
	QMatrix4x4 m_world;
	bool m_transparent;
	int width, height;
	int currentFrameNo;
	int outputNo;
};

#endif // DISPLAYGLWIDGET_H
