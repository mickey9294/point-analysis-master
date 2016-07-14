#include "myglwidget.h"

MyGLWidget::MyGLWidget(QWidget *parent)
	: QOpenGLWidget(parent)
{
	m = 3;
	m_model = new PCModel();
	m_xRot = 0;
	m_yRot = 0;
	m_zRot = 0;

	m_transparent = QCoreApplication::arguments().contains(QStringLiteral("--transparent"));
	if (m_transparent)
		setAttribute(Qt::WA_TranslucentBackground);

	setFocusPolicy(Qt::StrongFocus);
}

MyGLWidget::~MyGLWidget()
{

}

void MyGLWidget::setModel(PCModel *model)
{
	qDebug() << "setMeshModel, # of points =" << model->vertexCount();
	emit addDebugText("setPCModel, the number of points = " + QString::number(model->vertexCount()));

	PCModel *temp = m_model;
	m_model = model;
	connect(m_model, SIGNAL(onLabelsChanged()), this, SLOT(updateLabels()));
	delete(temp);

	/* Clear the current oriented bounding boxes */
	for (int i = 0; i < m_OBBs.size(); i++)
	{
		delete(m_OBBs[i]);
		m_OBBs.remove(i);
	}

	m_OBBs.clear();

	update();
}

void MyGLWidget::initializeGL()
{
	initializeOpenGLFunctions();
	glClearColor(255, 255, 255, m_transparent ? 0 : 1);

	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glEnable(GL_COLOR_MATERIAL);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //指定混合函数
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glShadeModel(GL_SMOOTH);
	init_light();
}

void MyGLWidget::init_light()
{
	GLfloat white_light[] = { 0.23, 0.23, 0.23, 1.0 };

	GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
	GLfloat light_position1[] = { 1.0, 1.0, -1.0, 0.0 };
	GLfloat light_position2[] = { 1.0, -1.0, 1.0, 0.0 };
	GLfloat light_position3[] = { 1.0, -1.0, -1.0, 0.0 };
	GLfloat light_position4[] = { -1.0, 1.0, 1.0, 0.0 };
	GLfloat light_position5[] = { -1.0, -1.0, 1.0, 0.0 };
	GLfloat light_position6[] = { -1.0, 1.0, -1.0, 0.0 };
	GLfloat light_position7[] = { -1.0, -1.0, -1.0, 0.0 };

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0); glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1); glLightfv(GL_LIGHT1, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT1, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT2, GL_POSITION, light_position2); glLightfv(GL_LIGHT2, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT2, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT3, GL_POSITION, light_position3); glLightfv(GL_LIGHT3, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT3, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT4, GL_POSITION, light_position4); glLightfv(GL_LIGHT4, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT4, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT5, GL_POSITION, light_position5); glLightfv(GL_LIGHT5, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT5, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT6, GL_POSITION, light_position6); glLightfv(GL_LIGHT6, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT6, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT7, GL_POSITION, light_position7); glLightfv(GL_LIGHT7, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT7, GL_SPECULAR, white_light);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHT4);
	//glEnable(GL_LIGHT5);
	//glEnable(GL_LIGHT6);
	//glEnable(GL_LIGHT7);
}

void MyGLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0, 1.0, 1.0, 1.0f);
	draw();
	glFlush();
	glFinish();
}

void MyGLWidget::draw()
{
	glPushMatrix();

	GLfloat no_mat[4] = { 0.0, 0.0, 0, 1 };
	GLfloat mat_diffuse[4] = { 0, 0, 0, 1 };		//r±íÊ¾´óÖµ£¬b±íÊ¾Ð¡Öµ
	GLfloat mat_specular[4] = { 0.5, 0.5, 0.5, 1 };
	float no_shininess[4] = { 1, 1, 1, 0 };
	glMaterialfv(GL_FRONT, GL_AMBIENT, no_mat);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, no_shininess);
	glMaterialfv(GL_FRONT, GL_EMISSION, no_mat);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/*QVector4D eye = QVector4D(0.0, 0.0, m, 1.0);
	gluLookAt(eye.x(), eye.y(), eye.z(), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);*/

	glTranslatef(m_model->getCenter().x(), m_model->getCenter().y(), m_model->getCenter().z());
	glRotatef(m_xRot, 1.0, 0.0, 0.0);
	glRotatef(m_yRot, 0.0, 1.0, 0.0);
	glRotatef(m_zRot, 0.0, 0.0, 1.0);
	glTranslatef(-m_model->getCenter().x(), -m_model->getCenter().y(), -m_model->getCenter().z());



	/* Draw the point cloud model m_model */
	glPointSize(2.0);
	glBegin(GL_POINTS);
	int nvertices = m_model->vertexCount();
	for (int i = 0; i < nvertices; i++)
	{
		const GLfloat *data = m_model->constData() + i * 9;
		GLfloat x = data[0];
		GLfloat y = data[1];
		GLfloat z = data[2];
		GLfloat nx = data[3];
		GLfloat ny = data[4];
		GLfloat nz = data[5];
		float cr = data[6];
		float cg = data[7];
		float cb = data[8];

		GLfloat color[4] = { cr, cg, cb, 1.0 };
		//glMaterialfv(GL_FRONT, GL_DIFFUSE, color);
		glColor4f(cr, cg, cb, 1.0);
		glNormal3f(nx, ny, nz);

		glVertex3f(m * x, m* y, m * z);
	}
	glEnd();

	/* Draw the oriented bounding boxes of the parts */
	int nboxes = m_OBBs.size();
	for (int i = 0; i < nboxes; i++)
	{
		int nfaces = m_OBBs[i]->facetCount();
		QVector3D color = m_OBBs[i]->getColor();
		glColor4f(color.x(), color.y(), color.z(), 0.5);

		glBegin(GL_TRIANGLES);
		for (int j = 0; j < nfaces; j++)
		{
			const GLfloat *data = m_OBBs[i]->constData() + j * 12;
			GLfloat v0x = data[0];
			GLfloat v0y = data[1];
			GLfloat v0z = data[2];
			GLfloat v1x = data[3];
			GLfloat v1y = data[4];
			GLfloat v1z = data[5];
			GLfloat v2x = data[6];
			GLfloat v2y = data[7];
			GLfloat v2z = data[8];
			GLfloat nx = data[9];
			GLfloat ny = data[10];
			GLfloat nz = data[11];

			//glMaterialfv(GL_FRONT, GL_DIFFUSE, color);

			glNormal3f(nx, ny, nz);

			glVertex3f(m * v0x, m * v0y, m * v0z);
			glVertex3f(m * v1x, m * v1y, m * v1z);
			glVertex3f(m * v2x, m * v2y, m * v2z);
		}

		glEnd();
	}

	//glColor4f(1.0, 0.0, 0.0, 0.5);
	//drawCylinder(0, 0.5, 0.0, 0, -0.5, 0, 0.7);
	glPopMatrix();
}

void MyGLWidget::resizeGL(int width, int height)
{
	GLfloat nRange = 5.0f;
	if (height == 0) {    // Prevent A Divide By Zero By  
		height = 1;    // Making Height Equal One  
	}
	glViewport(0, 0, width, height);    // Reset The Current Viewport  
	glMatrixMode(GL_PROJECTION);       // Select The Projection Matrix  
	glLoadIdentity();                  // Reset The Projection Matrix  

	if (width <= height)
		glOrtho(-nRange, nRange, -nRange*height / width, nRange*height / width, -nRange, nRange);
	else
		glOrtho(-nRange*width / height, nRange*width / height, -nRange, nRange, -nRange, nRange);
	glMatrixMode(GL_MODELVIEW);      // Select The Modelview Matrix  
	glLoadIdentity();
}

void MyGLWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastPos = event->pos();
	clickEvent = true;
}

void MyGLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_lastPos.x();
	int dy = event->y() - m_lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		setXRotation(m_xRot + 3 * dy);
		setYRotation(m_yRot + 3 * dx);
	}
	else if (event->buttons() & Qt::RightButton) {
		setXRotation(m_xRot + 3 * dy);
		setZRotation(m_zRot + 3 * dx);
	}
	m_lastPos = event->pos();
}

void MyGLWidget::wheelEvent(QWheelEvent *e)
{
	if (e->delta()>0)
		m -= 0.1f;
	if (e->delta()<0)
		m += 0.1f;
	if (m < 0.1)
		m = 0.1;
	update();
}

static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 * 16;
	while (angle > 360 * 16)
		angle -= 360 * 16;
}

void MyGLWidget::setXRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_xRot) {
		m_xRot = angle;
		update();
	}
}

void MyGLWidget::setYRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_yRot) {
		m_yRot = angle;
		update();
	}
}

void MyGLWidget::setZRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_zRot) {
		m_zRot = angle;
		update();
	}
}

void MyGLWidget::setOBBs(QVector<OBB *> obbs)
{
	emit addDebugText("Set oriented bounding boxes of the parts.");
	int size = obbs.size();

	/* Delete all the OBB in current m_OBBs */
	for (int i = 0; i < m_OBBs.size(); i++)
	{
		delete(m_OBBs[i]);
		m_OBBs.remove(i);
	}

	m_OBBs.resize(size);

	int i = 0;
	for (QVector<OBB *>::iterator it = obbs.begin(); it != obbs.end(); it++)
		m_OBBs[i++] = *it;

	update();
}

void MyGLWidget::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}

void MyGLWidget::updateLabels()
{
	emit addDebugText("Update the labels of points in GLWidget.");
	update();
}

void MyGLWidget::drawCylinder(float x0, float y0, float z0, float x1, float y1, float z1, double radius)
{
	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z);
	static GLUquadricObj *  quad_obj = NULL;
	if (quad_obj == NULL)
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle(quad_obj, GLU_LINE);
	gluQuadricNormals(quad_obj, GLU_SMOOTH);
	glPushMatrix();

	glTranslated(x0, y0, z0);
	//Calculate length. 
	double  length;
	length = sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z);
	if (length < 0.0001) {
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;
	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;
	double side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;
	length = sqrt(side_x*side_x + side_y*side_y + side_z*side_z);
	if (length < 0.0001) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;
	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;

	//Transformation matrix  
	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
		up_x, up_y, up_z, 0.0,
		dir_x, dir_y, dir_z, 0.0,
		0.0, 0.0, 0.0, 1.0 };
	glMultMatrixd(m);

	//Param for cylinder
	GLdouble slices = 8.0;
	GLdouble stack = 128.0;

	//gluQuadricDrawStyle(GLU_FILL);
	gluCylinder(quad_obj, radius, radius, bone_length, slices, stack);
	glPopMatrix();
}