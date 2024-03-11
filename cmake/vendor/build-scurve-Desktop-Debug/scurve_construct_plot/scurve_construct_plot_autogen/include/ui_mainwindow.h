/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_4;
    QFrame *frame;
    QGridLayout *gridLayout_3;
    QGridLayout *gridLayout;
    QGridLayout *gridLayout_2;
    QLabel *label_curpos;
    QLabel *label_3;
    QLabel *label_5;
    QPushButton *pushButton_jog_reverse;
    QSpacerItem *horizontalSpacer;
    QCheckBox *checkBox_finished;
    QLabel *label_6;
    QLabel *label;
    QPushButton *pushButton_jog_pos_rev;
    QPushButton *pushButton_pause;
    QLineEdit *lineEdit_fwd_tarpos;
    QLabel *label_4;
    QLineEdit *lineEdit_endvel;
    QPushButton *pushButton_jog_forward;
    QLineEdit *lineEdit_rev_tarpos;
    QLabel *label_curacc;
    QLabel *label_curvel;
    QPushButton *pushButton_jog_pos_fwd;
    QLabel *label_2;
    QPushButton *pushButton_jog_pause;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(793, 641);
        MainWindow->setStyleSheet(QString::fromUtf8("background-color: rgb(35, 35, 35);\n"
"color: rgb(255, 255, 255);\n"
""));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout_4 = new QGridLayout(centralWidget);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        frame = new QFrame(centralWidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        gridLayout_3 = new QGridLayout(frame);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));

        gridLayout_3->addLayout(gridLayout, 0, 0, 1, 1);


        gridLayout_4->addWidget(frame, 0, 0, 1, 1);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setSpacing(6);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_curpos = new QLabel(centralWidget);
        label_curpos->setObjectName(QString::fromUtf8("label_curpos"));

        gridLayout_2->addWidget(label_curpos, 3, 4, 1, 1);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 1, 3, 1, 1);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_2->addWidget(label_5, 4, 0, 1, 1);

        pushButton_jog_reverse = new QPushButton(centralWidget);
        pushButton_jog_reverse->setObjectName(QString::fromUtf8("pushButton_jog_reverse"));

        gridLayout_2->addWidget(pushButton_jog_reverse, 0, 0, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer, 0, 2, 1, 1);

        checkBox_finished = new QCheckBox(centralWidget);
        checkBox_finished->setObjectName(QString::fromUtf8("checkBox_finished"));

        gridLayout_2->addWidget(checkBox_finished, 3, 2, 1, 1);

        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_2->addWidget(label_6, 6, 0, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 0, 3, 1, 1);

        pushButton_jog_pos_rev = new QPushButton(centralWidget);
        pushButton_jog_pos_rev->setObjectName(QString::fromUtf8("pushButton_jog_pos_rev"));

        gridLayout_2->addWidget(pushButton_jog_pos_rev, 0, 1, 1, 1);

        pushButton_pause = new QPushButton(centralWidget);
        pushButton_pause->setObjectName(QString::fromUtf8("pushButton_pause"));

        gridLayout_2->addWidget(pushButton_pause, 3, 0, 1, 1);

        lineEdit_fwd_tarpos = new QLineEdit(centralWidget);
        lineEdit_fwd_tarpos->setObjectName(QString::fromUtf8("lineEdit_fwd_tarpos"));

        gridLayout_2->addWidget(lineEdit_fwd_tarpos, 4, 1, 1, 1);

        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_2->addWidget(label_4, 5, 0, 1, 1);

        lineEdit_endvel = new QLineEdit(centralWidget);
        lineEdit_endvel->setObjectName(QString::fromUtf8("lineEdit_endvel"));

        gridLayout_2->addWidget(lineEdit_endvel, 6, 1, 1, 1);

        pushButton_jog_forward = new QPushButton(centralWidget);
        pushButton_jog_forward->setObjectName(QString::fromUtf8("pushButton_jog_forward"));

        gridLayout_2->addWidget(pushButton_jog_forward, 1, 0, 1, 1);

        lineEdit_rev_tarpos = new QLineEdit(centralWidget);
        lineEdit_rev_tarpos->setObjectName(QString::fromUtf8("lineEdit_rev_tarpos"));

        gridLayout_2->addWidget(lineEdit_rev_tarpos, 5, 1, 1, 1);

        label_curacc = new QLabel(centralWidget);
        label_curacc->setObjectName(QString::fromUtf8("label_curacc"));

        gridLayout_2->addWidget(label_curacc, 1, 4, 1, 1);

        label_curvel = new QLabel(centralWidget);
        label_curvel->setObjectName(QString::fromUtf8("label_curvel"));
        label_curvel->setMinimumSize(QSize(100, 0));

        gridLayout_2->addWidget(label_curvel, 0, 4, 1, 1);

        pushButton_jog_pos_fwd = new QPushButton(centralWidget);
        pushButton_jog_pos_fwd->setObjectName(QString::fromUtf8("pushButton_jog_pos_fwd"));

        gridLayout_2->addWidget(pushButton_jog_pos_fwd, 1, 1, 1, 1);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 3, 3, 1, 1);

        pushButton_jog_pause = new QPushButton(centralWidget);
        pushButton_jog_pause->setObjectName(QString::fromUtf8("pushButton_jog_pause"));

        gridLayout_2->addWidget(pushButton_jog_pause, 2, 1, 1, 1);


        gridLayout_4->addLayout(gridLayout_2, 1, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 793, 23));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "scurve jog plot", nullptr));
        label_curpos->setText(QCoreApplication::translate("MainWindow", "0.000", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "curacc:", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "fwd_tarpos:", nullptr));
        pushButton_jog_reverse->setText(QCoreApplication::translate("MainWindow", "jog velocity reverse", nullptr));
        checkBox_finished->setText(QCoreApplication::translate("MainWindow", "finished", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "velocity end:", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "curvel:", nullptr));
        pushButton_jog_pos_rev->setText(QCoreApplication::translate("MainWindow", "jog position reverse", nullptr));
        pushButton_pause->setText(QCoreApplication::translate("MainWindow", "pause timer", nullptr));
        lineEdit_fwd_tarpos->setText(QCoreApplication::translate("MainWindow", "100", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "rev_tarpos:", nullptr));
        lineEdit_endvel->setText(QCoreApplication::translate("MainWindow", "2", nullptr));
        pushButton_jog_forward->setText(QCoreApplication::translate("MainWindow", "jog velocity forward", nullptr));
        lineEdit_rev_tarpos->setText(QCoreApplication::translate("MainWindow", "-100", nullptr));
        label_curacc->setText(QCoreApplication::translate("MainWindow", "0.000", nullptr));
        label_curvel->setText(QCoreApplication::translate("MainWindow", "0.000", nullptr));
        pushButton_jog_pos_fwd->setText(QCoreApplication::translate("MainWindow", "jog position forward", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "curpos:", nullptr));
        pushButton_jog_pause->setText(QCoreApplication::translate("MainWindow", "jog pause", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
