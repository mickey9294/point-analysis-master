/********************************************************************************
** Form generated from reading UI file 'progressdialog1.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PROGRESSDIALOG1_H
#define UI_PROGRESSDIALOG1_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ProgressDialog1
{
public:
    QLabel *progressMessage1;
    QProgressBar *progressBar1;

    void setupUi(QWidget *ProgressDialog1)
    {
        if (ProgressDialog1->objectName().isEmpty())
            ProgressDialog1->setObjectName(QStringLiteral("ProgressDialog1"));
        ProgressDialog1->resize(448, 94);
        progressMessage1 = new QLabel(ProgressDialog1);
        progressMessage1->setObjectName(QStringLiteral("progressMessage1"));
        progressMessage1->setGeometry(QRect(20, 10, 411, 31));
        progressBar1 = new QProgressBar(ProgressDialog1);
        progressBar1->setObjectName(QStringLiteral("progressBar1"));
        progressBar1->setGeometry(QRect(20, 50, 421, 31));
        progressBar1->setValue(0);

        retranslateUi(ProgressDialog1);

        QMetaObject::connectSlotsByName(ProgressDialog1);
    } // setupUi

    void retranslateUi(QWidget *ProgressDialog1)
    {
        ProgressDialog1->setWindowTitle(QApplication::translate("ProgressDialog1", "ProgressDialog1", 0));
        progressMessage1->setText(QApplication::translate("ProgressDialog1", "<html><head/><body><p><span style=\" font-size:12pt;\">TextLabel</span></p></body></html>", 0));
    } // retranslateUi

};

namespace Ui {
    class ProgressDialog1: public Ui_ProgressDialog1 {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PROGRESSDIALOG1_H
