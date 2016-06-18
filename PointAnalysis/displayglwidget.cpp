#include "displayglwidget.h"

DisplayGLWidget::DisplayGLWidget(QWidget *parent)
	: QOpenGLWidget(parent), m_program(0), currentFrameNo(0),
	m_xRot(0), m_yRot(0), m_zRot(0), outputNo(0), m_color(0.39, 1.0, 0.0), m_cameraPositionZ(0)
{
	m_model = new PCModel();
	m_obbVbos.resize(4);
	m_OBBs.resize(4);
	for (int i = 0; i < 4; i++)
		m_OBBs[i] = new OBB();
	m_core = QCoreApplication::arguments().contains(QStringLiteral("--coreprofile"));
	m_transparent = QCoreApplication::arguments().contains(QStringLiteral("--transparent"));
	if (m_transparent)
		setAttribute(Qt::WA_TranslucentBackground);
}

DisplayGLWidget::~DisplayGLWidget()
{
	cleanup();
}

void DisplayGLWidget::cleanup()
{
	makeCurrent();
	m_meshModelVbo.destroy();
	delete m_program;
	m_program = 0;
	delete(m_model);
	for (int i = 0; i < m_OBBs.size(); i++)
		delete(m_OBBs[i]);
	m_OBBs.clear();
	doneCurrent();
}

void DisplayGLWidget::clearScene()
{
	m_model->clear();
	m_xRot = 0;
	m_yRot = 0;
	m_zRot = 0;
	currentFrameNo = 0;
}

static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 * 16;
	while (angle > 360 * 16)
		angle -= 360 * 16;
}

void DisplayGLWidget::setXRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_xRot) {
		m_xRot = angle;
		update();
	}
}

void DisplayGLWidget::setYRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_yRot) {
		m_yRot = angle;
		update();
	}
}

void DisplayGLWidget::setZRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_zRot) {
		m_zRot = angle;
		update();
	}
}

void DisplayGLWidget::setModel(PCModel *model)
{
	qDebug() << "setMeshModel, # of points =" << model->vertexCount();
	emit addDebugText("setPCModel, the number of points = " + QString::number(model->vertexCount()));

	PCModel *temp = m_model;
	m_model = model;
	delete(temp);

	/*for (int i = 0; i < m_obbVbos.size(); i++)
		m_obbVbos[i].release();
	m_obbVbos.clear();*/

	setupVertexAttribs();
	update();
}

PCModel * DisplayGLWidget::getModel()
{
	return (m_model);
}

static const char *vertexShaderSourceCore =
"#version 150\n"
"in vec4 vertex;\n"
"in vec3 normal;\n"
"in vec3 color;\n"
"out vec3 vert;\n"
"out vec3 vertNormal;\n"
"out vec3 vertColor;\n"
"uniform mat4 projMatrix;\n"
"uniform mat4 mvMatrix;\n"
"uniform mat3 normalMatrix;\n"
"void main() {\n"
"   vert = vertex.xyz;\n"
"   vertNormal = normalMatrix * normal;\n"
"   vertColor = color;\n"
"   gl_Position = projMatrix * mvMatrix * vertex;\n"
"}\n";

static const char *fragmentShaderSourceCore =
"#version 150\n"
"in highp vec3 vert;\n"
"in highp vec3 vertNormal;\n"
"in highp vec3 vertColor;\n"
"out highp vec4 fragColor;\n"
"uniform highp vec3 lightPos;\n"
"void main() {\n"
"   highp vec3 L = normalize(lightPos - vert);\n"
"   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
"   highp vec3 color = vec3(0.5, 0.5, 0);\n"
"   highp vec3 col = clamp(vertColor * 0.2 + vertColor * 0.8 * NL, 0.0, 1.0);\n"
"   fragColor = vec4(col, 1.0);\n"
"}\n";

static const char *vertexShaderSource =
"attribute vec4 vertex;\n"
"attribute vec3 normal;\n"
"attribute vec4 color;\n"
"varying vec3 vert;\n"
"varying vec3 vertNormal;\n"
"varying vec4 vertColor;\n"
"uniform mat4 projMatrix;\n"
"uniform mat4 mvMatrix;\n"
"uniform mat3 normalMatrix;\n"
"void main() {\n"
"   vert = vertex.xyz;\n"
"   vertColor = color;\n"
"   vertNormal = normalMatrix * normal;\n"
"   gl_Position = projMatrix * mvMatrix * vertex;\n"
"}\n";

static const char *fragmentShaderSource =
"varying highp vec3 vert;\n"
"varying highp vec3 vertNormal;\n"
"varying highp vec4 vertColor;\n"
"uniform highp vec3 lightPos;\n"
"void main() {\n"
"   highp vec3 L = normalize(lightPos - vert);\n"
"   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
"   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
"   highp vec3 col = clamp(vertColor.xyz * 0.2 + vertColor.xyz * 0.8 * NL, 0.0, 1.0);\n"
"   gl_FragColor = vec4(col, vertColor.w);\n"
"}\n";

void DisplayGLWidget::initializeGL()
{
	// In this example the widget's corresponding top-level window can change
	// several times during the widget's lifetime. Whenever this happens, the
	// QOpenGLWidget's associated context is destroyed and a new one is created.
	// Therefore we have to be prepared to clean up the resources on the
	// aboutToBeDestroyed() signal, instead of the destructor. The emission of
	// the signal will be followed by an invocation of initializeGL() where we
	// can recreate all resources.

	initializeOpenGLFunctions();
	glClearColor(255, 255, 255, m_transparent ? 0 : 1);

	m_program = new QOpenGLShaderProgram;
	m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, m_core ? vertexShaderSourceCore : vertexShaderSource);
	m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, m_core ? fragmentShaderSourceCore : fragmentShaderSource);
	m_program->bindAttributeLocation("vertex", 0);
	m_program->bindAttributeLocation("normal", 1);
	m_program->bindAttributeLocation("color", 2);
	m_program->link();

	m_program->bind();
	m_projMatrixLoc = m_program->uniformLocation("projMatrix");
	m_mvMatrixLoc = m_program->uniformLocation("mvMatrix");
	m_normalMatrixLoc = m_program->uniformLocation("normalMatrix");
	m_lightPosLoc = m_program->uniformLocation("lightPos");
	//m_colorLoc = m_program->uniformLocation("vertColor");

	// Create a vertex array object. In OpenGL ES 2.0 and OpenGL 2.x
	// implementations this is optional and support may not be present
	// at all. Nonetheless the below code works in all cases and makes
	// sure there is a VAO when one is needed.
	m_vao.create();
	QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

	m_camera.setToIdentity();
	QVector3D eye(0, 0, 8.0);
	QVector3D up(0, 1.0, 0);
	QVector3D center(0, 0, 0.0);
	m_camera.lookAt(eye, center, up);

	// Store the vertex attribute bindings for the program.
	setupVertexAttribs();

	// Light position is fixed.
	m_program->setUniformValue(m_lightPosLoc, QVector3D(0, 0, 70));

	m_program->release();
}

void DisplayGLWidget::setupVertexAttribs()
{
	// Setup our vertex buffer object.
	m_meshModelVbo.create();
	m_meshModelVbo.bind();
	m_meshModelVbo.allocate(m_model->constData(), m_model->count() * sizeof(GLfloat));
	m_meshModelVbo.bind();
	/*QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	f->glEnableVertexAttribArray(0);
	f->glEnableVertexAttribArray(1);
	f->glEnableVertexAttribArray(2);
	f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 10 * sizeof(GLfloat), 0);
	f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 10 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
	f->glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 10 * sizeof(GLfloat), reinterpret_cast<void *>(6 * sizeof(GLfloat)));*/

	m_meshModelVbo.release();

	for (int i = 0; i < m_obbVbos.size(); i++)
	{
		m_obbVbos[i].create();
		m_obbVbos[i].bind();
		m_obbVbos[i].allocate(m_OBBs[i]->constData(), m_OBBs[i]->count() * sizeof(float));
		m_obbVbos[i].bind();
		m_obbVbos[i].release();
	}
}

void DisplayGLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	/* paint 1st object */
	m_meshModelVbo.bind();

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

	f->glEnableVertexAttribArray(0);
	f->glEnableVertexAttribArray(1);
	f->glEnableVertexAttribArray(2);
	f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), 0);
	f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
	f->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), reinterpret_cast<void *>(6 * sizeof(GLfloat)));

	m_world.setToIdentity();
	m_world.translate(m_model->getCenter().x(), m_model->getCenter().y(), m_model->getCenter().z());
	m_world.rotate(m_xRot / 16.0f, 1, 0, 0);
	m_world.rotate(m_yRot / 16.0f, 0, 1, 0);
	m_world.rotate(m_zRot / 16.0f, 0, 0, 1);
	m_world.translate(-m_model->getCenter().x(), -m_model->getCenter().y(), -m_model->getCenter().z());

	QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
	m_program->bind();
	m_program->setUniformValue(m_projMatrixLoc, m_proj);
	m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);
	//m_program->setUniformValue(m_colorLoc, m_color);

	QMatrix3x3 normalMatrix = m_world.normalMatrix();
	m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);
	glPointSize(2.0);
	glDrawArrays(GL_POINTS, 0, m_model->vertexCount());

	//glFinish();
	//m_vao.release();

	m_program->release();

	/* Draw the oriented bounding boxes for parts */
	for (int i = 0; i < m_obbVbos.size(); i++)
	{
		m_obbVbos[i].bind();

		//delete(f);
		f = QOpenGLContext::currentContext()->functions();
		f->glEnableVertexAttribArray(0);
		f->glEnableVertexAttribArray(1);
		f->glEnableVertexAttribArray(2);
		f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), 0);
		f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
		f->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), reinterpret_cast<void *>(6 * sizeof(GLfloat)));

		m_program->bind();
		m_program->setUniformValue(m_projMatrixLoc, m_proj);
		m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);

		m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);
		glDrawArrays(GL_TRIANGLES, 0, m_OBBs[i]->vertexCount());
		//glFinish();
		m_program->release();
	}
}

void DisplayGLWidget::resizeGL(int w, int h)
{
	if (width != w)
		width = w;
	if (height != h)
		height = h;
	m_proj.setToIdentity();
	m_proj.perspective(30.0f, GLfloat(w) / h, 2.0f, 20.0f);
}


void DisplayGLWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastPos = event->pos();
}

void DisplayGLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_lastPos.x();
	int dy = event->y() - m_lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		setXRotation(m_xRot + 8 * dy);
		setYRotation(m_yRot + 8 * dx);
	}
	else if (event->buttons() & Qt::RightButton) {
		setXRotation(m_xRot + 8 * dy);
		setZRotation(m_zRot + 8 * dx);
	}
	m_lastPos = event->pos();
}

void DisplayGLWidget::wheelEvent(QWheelEvent *e)
{
	if (e->delta() > 0)
		m_camera.translate(0, 0, -0.20);
		//m_cameraPositionZ -= 0.1f;
	if (e->delta()<0)
		m_camera.translate(0, 0, 0.20);
		//m_cameraPositionZ += 0.1f;

	update();
}

using namespace std;
void DisplayGLWidget::outputPointCloud()
{
	emit(startOutput(m_model, outputNo));
	string downsample_cmd = "CloudCompare -O ../point_clouds/pc" + to_string(outputNo) + ".ply -AUTO_SAVE ON -NO_TIMESTAMP -C_EXPORT_FMT PLY -SS SPATIAL 1.6";
	qDebug() << "Downsampling the point cloud...";
	system(downsample_cmd.c_str());
	qDebug() << "Downsample point cloud done.";
	emit(startRecon(outputNo));
}

void DisplayGLWidget::updateLabels()
{
	emit addDebugText("Update the labels of points in GLWidget.");
	setupVertexAttribs();
	update();
}

void DisplayGLWidget::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}

void DisplayGLWidget::setOBBs(QVector<OBB *> obbs)
{
	emit addDebugText("Set oriented bounding boxes of the parts.");
	int size = obbs.size();
	/*if (m_obbVbos.size() != size)
	{
		m_OBBs.resize(size);
		m_obbVbos.resize(size);
	}*/
	int i = 0;
	for (QVector<OBB *>::iterator it = obbs.begin(); it != obbs.end(); it++)
	{
		//OBB *temp_to_delete = m_OBBs[i];
		m_OBBs[i++] = *it;
		//delete(temp_to_delete);
	}

	setupVertexAttribs();
	update();
}