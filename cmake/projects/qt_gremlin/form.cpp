#include "form.h"
#include "ui_form.h"

Form::Form(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Form)
{
    ui->setupUi(this);
}

Form::~Form()
{
    delete ui;
}

void Form::update_dro(double x, double y, double z, double dtgx, double dtgy, double dtgz, double vel){
     ui->label_dro_x->setText(QString::number(x,'f',3));
     ui->label_dro_y->setText(QString::number(y,'f',3));
     ui->label_dro_z->setText(QString::number(z,'f',3));

     ui->label_dro_dtg_x->setText(QString::number(dtgx,'f',3));
     ui->label_dro_dtg_y->setText(QString::number(dtgy,'f',3));
     ui->label_dro_dtg_z->setText(QString::number(dtgz,'f',3));

     ui->label_dro_vel->setText(QString::number(vel*60,'f',3));
}

void Form::reset_btn_press(){
    fit_all=false;
}

bool Form::is_fit_all(){
    return fit_all;
}

void Form::on_toolButton_fit_all_pressed()
{
    fit_all=true;
}
