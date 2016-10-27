#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QOpenGLWidget>
#include <QDebug>
#include <QString>
#include <QtOpenGL>
#include <QtGui>
#include "pcmodel.h"
#include "obb.h"
#include <Windows.h>
#include <GL/glu.h>
#include <Eigen\Core>
#include "meshmodel.h"
#include "meshpcmodel.h"
#include "PartsStructure.h"

#ifndef PI
#define PI 3.1415926536
#endif

class MyGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	MyGLWidget(QWidget *parent = 0);
	~MyGLWidget();

	int xRotation() const { return m_xRot; }
	int yRotation() const { return m_yRot; }
	int zRotation() const { return m_zRot; }

	void resetView();
	Model * getModel() { return m_model; }

	public slots:
	void setDrawSymmetryPlanes(int state);
	void setDrawSymmetryAxes(int state);
	void setDrawOBBs(int state);
	void setDrawOBBsAxes(int state);

signals:
	void xRotationChanged(int angle);
	void yRotationChanged(int angle);
	void zRotationChanged(int angle);
	void addDebugText(QString text);

	public slots:
	void setXRotation(int angle);
	void setYRotation(int angle);
	void setZRotation(int angle);
	void setModel(Model *m);
	void setOBBs(QVector<OBB *> obbs);
	void onDebugTextAdded(QString text);
	void updateLabels();
	void setSamples(Samples_Vec samples);
	void setPartsStructure(Parts_Structure_Pointer structure);
	void rotateModel(float angle, float x, float y, float z);

protected:
	void initializeGL();
	void paintGL();
	void init_light();
	void resizeGL(int w, int h);
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	//void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *e);

	private slots:
	void draw();

private:
	void normalizeAngle(int &angle);
	void drawSphere(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat radius, GLfloat M, GLfloat N);
	void ProcessPicks(GLint nPicks, GLuint pickBuffer[]);
	void drawCylinder(float x0, float y0, float z0, float x1, float y1, float z1, double radius);
	bool show_parts_structure;

	static GLint _colorList[12][3];

	GLfloat m_xRot;
	GLfloat m_yRot;
	GLfloat m_zRot;
	bool m_transparent;
	float m;
	Model * m_model;
	QVector<OBB *> m_OBBs;
	Samples_Vec m_samples;
	Parts_Structure_Pointer m_parts_structure;

	QPoint m_lastPos;
	bool clickEvent;
	bool m_draw_obbs;
};

#endif // MYGLWIDGET_H