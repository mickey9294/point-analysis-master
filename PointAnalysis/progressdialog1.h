#ifndef PROGRESSDIALOG1_H
#define PROGRESSDIALOG1_H

#include <QDialog>
#include "ui_progressdialog1.h"

class ProgressDialog1 : public QDialog
{
	Q_OBJECT

public:
	ProgressDialog1(QWidget *parent = 0);
	~ProgressDialog1();

	public slots:
	void setProgressValue(int value);
	void setMessage(const char *msg);

private:
	Ui::ProgressDialog1 ui;
};

#endif // PROGRESSDIALOG1_H
