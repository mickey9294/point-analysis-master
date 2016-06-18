#include "progressdialog1.h"

ProgressDialog1::ProgressDialog1(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

ProgressDialog1::~ProgressDialog1()
{

}

void ProgressDialog1::setProgressValue(int value)
{
	ui.progressBar1->setValue(value);
}

void ProgressDialog1::setMessage(const char *msg)
{
	ui.progressMessage1->setText(msg);
}