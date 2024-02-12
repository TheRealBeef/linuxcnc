#include "mainwindow.h"
#include "./ui_mainwindow.h"

std::string oldfile="";
double oldx=0,oldy=0,oldz=0;
double jog_speed=0;
bool lock_feed_override=0, lock_maxvel=0, lock_spindle_override=0, lock_rapid_override=0, lock_flood=0, lock_mist=0, lock_spindle=0;

extern std::vector<cad_data> runInterpreter(std::string filename);

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // occ = new Opencascade();
    occ = new  OcctQtViewer();

    //! Add gridlayout on top of the occ widget.
    QGridLayout *layout=new QGridLayout(occ);

    //! Add the controls into the occ gridlayout..
    layout->addWidget(form);

    ui->gridLayout_occ->addWidget(occ);
    occ->create_tp_cone();

    editor = new QGCodeEditor();
    ui->gridLayout_gcode->addWidget(editor);
    editor->setStyleSheet("background-color: rgb(255, 255, 255);\ncolor: rgb(0, 0, 0);");

    jog_speed=ui->horizontalSlider_jog_speed->value();

    //! Create a nml interface. From here we can see wich gcode file to load.
    //stat = new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "xemc", EMC2_DEFAULT_NMLFILE);

    //! This activates a screen update when robot is moving and screen needs to be updated automaticly.
    connect(timer, &QTimer::timeout, this, &MainWindow::update);
    timer->start(50);
}

MainWindow::~MainWindow()
{
    delete ui;
}

int s=0;

void MainWindow::make_fillets(double radius){

    // occ->add_shapevec(draw_primitives().draw_3d_pc_circle({0,0,0},{0,0,1},150));

    std::vector<Handle(AIS_Shape)> aFilletShapeVec, tempVec;

    for(uint i=0; i<occ->aShapeVec.size()-1; i++){
        Handle(AIS_Shape) aShapeA,aShapeB;


        //! Line-line
        if(draw_primitives().get_shapetype(occ->aShapeVec.at(i))==1 && draw_primitives().get_shapetype(occ->aShapeVec.at(i+1))==1){
            //! Create fillet.
            //            if(draw_primitives().draw_3d_line_line_fillet(occ->aShapeVec.at(i),occ->aShapeVec.at(i+1),radius,aShape)){
            //                occ->add_shapevec(aShape);
            //            }
        }
        //! Line-arc
        if(draw_primitives().get_shapetype(occ->aShapeVec.at(i))==1 && draw_primitives().get_shapetype(occ->aShapeVec.at(i+1))==2){

            // draw_primitives().draw_3d_line_arc_offset_lines(occ->aShapeVec.at(i),occ->aShapeVec.at(i+1),radius,aShapeA,aShapeB);
            // occ->add_shapevec(aShapeA);
            // occ->add_shapevec(aShapeB);

            aFilletShapeVec=draw_primitives().create_line_arc_intersections(occ->aShapeVec.at(i),occ->aShapeVec.at(i+1),radius);

            for(uint k=0; k<aFilletShapeVec.size(); k++){
                // occ->show_shape(aFilletShapeVec.at(k));
                tempVec.push_back(aFilletShapeVec.at(k));
            }

            //! Create fillet.
            //            if(draw_primitives().draw_3d_line_arc_fillet_conventional(occ->aShapeVec.at(i),occ->aShapeVec.at(i+1),radius,aShape)){
            //                occ->add_shapevec(aShape);
            //            }
        }
        //! Arc-line
        if(draw_primitives().get_shapetype(occ->aShapeVec.at(i))==2 && draw_primitives().get_shapetype(occ->aShapeVec.at(i+1))==1){
            //! Create fillet.
            //if(draw_primitives().draw_3d_arc_line_fillet_conventional(occ->aShapeVec.at(i),occ->aShapeVec.at(i+1),radius,aShape)){
            //    occ->add_shapevec(aShape);
            //}
        }
    }

    for(uint i=0; i<tempVec.size(); i++){
        occ->add_shapevec(tempVec.at(i));
    }
}

void MainWindow::update(){

    //! Update nml.
    nml->update();

    //! Loaded gcode file.
    if(oldfile!=nml->theStatus.filename && nml->theStatus.filename.size()>0){
        load_cad_data_from_interpreter(nml->theStatus.filename);
        load_gcode_text(nml->theStatus.filename);

        // timer->stop();
        // make_fillets(5);
        // timer->start(50);

        oldfile=nml->theStatus.filename;
    }

    //! Update xyz dro.
    form->update_dro(nml->theStatus.x,
                     nml->theStatus.y,
                     nml->theStatus.z,
                     nml->theStatus.dtgx,
                     nml->theStatus.dtgy,
                     nml->theStatus.dtgz,
                     nml->theStatus.current_velocity,
                     nml->theStatus.homed_x,
                     nml->theStatus.homed_y,
                     nml->theStatus.homed_z);

    //! Update cad cone.
    occ->translate_tp_cone(nml->theStatus.x,
                           nml->theStatus.y,
                           nml->theStatus.z,0,-0.5*M_PI,0);


    editor->highlightLine(nml->theStatus.motion_line);

    if(!lock_feed_override){
        ui->horizontalSlider_feed_override->setValue(nml->theStatus.feed_override*100);
    }

    if(!lock_rapid_override){
        ui->horizontalSlider_rapid_override->setValue(nml->theStatus.rapid_override*100);
    }

    if(!lock_spindle_override){
        ui->horizontalSlider_spindle_override->setValue(nml->theStatus.spindle_override*100);
    }

    if(!lock_maxvel){
        ui->horizontalSlider_max_velocity->setValue(nml->theStatus.max_velocity*60);
    }

    if(!lock_mist){
        ui->checkBox_mist->setChecked(nml->theStatus.mist);
    }

    if(!lock_flood){
        ui->checkBox_flood->setChecked(nml->theStatus.flood);
    }

    if(nml->theStatus.spindle_speed==0){
        ui->toolButton_spindle_stop->setChecked(true);
    } else {
        ui->toolButton_spindle_stop->setChecked(false);
    }
    if(nml->theStatus.spindle_speed>0){
        ui->toolButton_spindle_cw->setChecked(true);
    } else {
        ui->toolButton_spindle_cw->setChecked(false);
    }
    if(nml->theStatus.spindle_speed<0){
        ui->toolButton_spindle_ccw->setChecked(true);
    } else {
        ui->toolButton_spindle_ccw->setChecked(false);
    }

    ui->label_feed_override->setText(QString::number(nml->theStatus.feed_override*100,'f',0)+QString(" %"));
    ui->label_rapid_override->setText(QString::number(nml->theStatus.rapid_override*100,'f',0)+QString(" %"));
    ui->label_spindle_override->setText(QString::number(nml->theStatus.spindle_override*100,'f',0)+QString(" %"));
    ui->label_max_velocity->setText(QString::number(nml->theStatus.max_velocity*60,'f',0)+QString(" mm/min"));

    ui->label_spindle_rpm->setText(QString::number(nml->theStatus.spindle_speed/60,'f',1)+QString(" rps"));

    ui->label_jog_speed->setText(QString::number(ui->horizontalSlider_jog_speed->value(),'f',0)+QString(" mm/min"));

    //! Set active gcodes.
    ui->plainTextEdit_active_gcodes->clear();
    ui->plainTextEdit_active_gcodes->appendPlainText(nml->theStatus.task_active_gcodes_string);

    if(nml->theStatus.machine_on){
        ui->toolButton_machine_on->setChecked(true);
        ui->label_machine_on_status->setStyleSheet("background-color: rgb(0, 170, 0);");

    } else {
        ui->toolButton_machine_on->setChecked(false);
        ui->label_machine_on_status->setStyleSheet("background-color: rgb(170, 85, 0);");
    }
    if(nml->theStatus.estop){
        ui->toolButton_emergency->setChecked(true);
        ui->label_emergency_status->setStyleSheet("background-color: rgb(170, 0, 0);");
        ui->label_machine_status->setText("ESTOP");
    } else {
        ui->label_emergency_status->setStyleSheet("background-color: rgb(0, 170, 0);");
        ui->label_machine_status->setText("OFF");
    }

    if(nml->theStatus.machine_on && !nml->theStatus.estop){
        ui->label_machine_status->setText("ON");
    }

    //! To update tp moves.
    occ->redraw();
}

//! Todo colorize rapids little different.
void MainWindow::load_cad_data_from_interpreter(std::string filename)
{
    occ->clear_shapevec();

    std::vector<cad_data> cadvec=runInterpreter(filename);
    s=cadvec.size();

    for(uint i=0; i<cadvec.size(); i++){

        cad_data d=cadvec.at(i);

        if(d.type==1 /*G1*/ || d.type==3 /*G0*/){ //! Draw opencascade line.
            gp_Pnt p0(oldx,oldy,oldz);
            gp_Pnt p1(d.pose.tran.x, d.pose.tran.y, d.pose.tran.z);

            if(d.type==1){
                occ->add_shapevec(draw_primitives().colorize( draw_primitives().draw_3d_line(p0,p1),Quantity_NOC_GRAY50,0) );
            }
            if(d.type==3){
                occ->add_shapevec(draw_primitives().colorize( draw_primitives().draw_3d_line(p0,p1),Quantity_NOC_GRAY15,0) );
            }
        }
        if(d.type==2){ //! Draw opencascade arc.

            if(d.plane==1){ //! XY plane
                // std::cout<<"plane 1, XY"<<std::endl; //! See 3dtest.ngc how to check this planes.

                gp_Pnt p0(oldx,oldy,oldz);
                gp_Pnt p1(d.pose.tran.x, d.pose.tran.y,  d.pose.tran.z);
                gp_Pnt pc(d.cx,d.cy, d.pose.tran.z);
                /*
                std::cout<<"arc start  p0x:"<<p0.X()<<" p0y:"<<p0.Y()<<" p0z:"<<p0.Z()<<std::endl;
                std::cout<<"arc end    p1x:"<<p1.X()<<" p1y:"<<p1.Y()<<" p1z:"<<p1.Z()<<std::endl;
                std::cout<<"arc center pcx:"<<pc.X()<<" pcy:"<<pc.Y()<<" pcz:"<<pc.Z()<<std::endl;
                std::cout<<""<<std::endl;*/

                if(d.rotation==-1){
                    occ->add_shapevec(draw_primitives().colorize( draw_primitives().draw_3d_pc_arc(p1,p0,pc,0,0,1),Quantity_NOC_GRAY50,0) );
                } else {
                    occ->add_shapevec(draw_primitives().colorize( draw_primitives().draw_3d_pc_arc(p0,p1,pc,0,0,1),Quantity_NOC_GRAY50,0) );
                }
            }
            if(d.plane==2){ //!
                // std::cout<<"plane 2, YZ"<<std::endl;

                gp_Pnt p0(oldx,oldy,oldz);
                gp_Pnt p1(d.pose.tran.z,d.pose.tran.x, d.pose.tran.y);
                gp_Pnt pc(d.pose.tran.z,d.cx,d.cy);
                /*
                std::cout<<"arc start  p0x:"<<p0.X()<<" p0y:"<<p0.Y()<<" p0z:"<<p0.Z()<<std::endl;
                std::cout<<"arc end    p1x:"<<p1.X()<<" p1y:"<<p1.Y()<<" p1z:"<<p1.Z()<<std::endl;
                std::cout<<"arc center pcx:"<<pc.X()<<" pcy:"<<pc.Y()<<" pcz:"<<pc.Z()<<std::endl;
                std::cout<<""<<std::endl;*/

                if(d.rotation==-1){
                    occ->add_shapevec(draw_primitives().colorize( draw_primitives().draw_3d_pc_arc(p1,p0,pc,1,0,0),Quantity_NOC_GRAY50,0) );
                } else {
                    occ->add_shapevec(draw_primitives().colorize( draw_primitives().draw_3d_pc_arc(p0,p1,pc,1,0,0),Quantity_NOC_GRAY50,0) );
                }
            }
            if(d.plane==3){ //! Y swap Z.
                // std::cout<<"plane 3, XZ"<<std::endl;

                gp_Pnt p0(oldx,oldy,oldz);
                gp_Pnt p1(d.pose.tran.x, d.pose.tran.z,d.pose.tran.y );
                gp_Pnt pc(d.cx, d.pose.tran.z,d.cy);
                /*
                std::cout<<"arc start  p0x:"<<p0.X()<<" p0y:"<<p0.Y()<<" p0z:"<<p0.Z()<<std::endl;
                std::cout<<"arc end    p1x:"<<p1.X()<<" p1y:"<<p1.Y()<<" p1z:"<<p1.Z()<<std::endl;
                std::cout<<"arc center pcx:"<<pc.X()<<" pcy:"<<pc.Y()<<" pcz:"<<pc.Z()<<std::endl;
                std::cout<<""<<std::endl;*/

                if(d.rotation==-1){
                    occ->add_shapevec(draw_primitives().colorize( draw_primitives().draw_3d_pc_arc(p1,p0,pc,0,1,0),Quantity_NOC_GRAY50,0) );
                } else {
                    occ->add_shapevec(draw_primitives().colorize( draw_primitives().draw_3d_pc_arc(p0,p1,pc,0,1,0),Quantity_NOC_GRAY50,0) );
                }
            }
        }

        oldx=d.pose.tran.x;
        oldy=d.pose.tran.y;
        oldz=d.pose.tran.z;
    }

    occ->fit_all();
}

int MainWindow::digits(int nr)
{
    int digits = 0;
    if (nr < 0) digits = 1; // remove this line if '-' counts as a digit
    while (nr) {
        nr /= 10;
        digits++;
    }
    return digits;
}

#include "QMessageBox"
#include "QTextStream"
void MainWindow::load_gcode_text(std::string filename){

    editor->clear();

    QFile file(filename.c_str());
    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(0, "error", file.errorString());
    }

    QTextStream in(&file);

    while(!in.atEnd()) {

        QString line;
        line.append(in.readLine());
        editor->appendNewPlainText(line);
    }
    file.close();
}

void MainWindow::on_toolButton_fit_all_pressed()
{
    occ->fit_all();
}

void MainWindow::on_toolButton_view_front_pressed()
{
    occ->set_view_front();
}

void MainWindow::on_toolButton_view_back_pressed()
{
    occ->set_view_back();
}

void MainWindow::on_toolButton_view_top_pressed()
{
    occ->set_view_top();
}

void MainWindow::on_toolButton_view_bottom_pressed()
{
    occ->set_view_bottom();
}

void MainWindow::on_toolButton_view_left_pressed()
{
    occ->set_view_left();
}

void MainWindow::on_toolButton_view_right_pressed()
{
    occ->set_view_right();
}

void MainWindow::on_toolButton_view_3d_pressed()
{
    occ->set_view_3d();
}

void MainWindow::on_toolButton_zoom_min_pressed()
{
    occ->zoom_min();
}

void MainWindow::on_toolButton_zoom_plus_pressed()
{
    occ->zoom_plus();
}

void MainWindow::on_toolButton_stop_pressed()
{
    nml->stop();
}

void MainWindow::on_toolButton_pause_toggled(bool checked)
{
    if(checked){
        nml->pause();
    } else {
        nml->resume();
    }
}

void MainWindow::on_toolButton_run_pressed()
{
    nml->run(1);
}

void MainWindow::on_toolButton_file_open_pressed()
{
    nml->mode_auto();
    nml->close();
    QString filename = QFileDialog::getOpenFileName(this, tr("Open File"), "/home/jana", tr("Image Files (*.txt *.ngc)"));
    nml->load(filename.toStdString());
}

void MainWindow::on_toolButton_reload_pressed()
{
    nml->mode_auto();
    std::string filename=nml->theStatus.filename;
    nml->close();
    nml->load(filename);

}

void MainWindow::on_toolButton_machine_on_pressed()
{
    if(ui->toolButton_machine_on->isChecked()){
        nml->machine_off();
    } else {
        nml->machine_on();
    }
}

void MainWindow::on_toolButton_emergency_pressed()
{
    if(ui->toolButton_emergency->isChecked()){
        nml->estop_reset();
    } else {
        nml->estop();
    }
}

void MainWindow::on_horizontalSlider_max_velocity_sliderMoved(int position)
{
    nml->setMaxVelocity(position/60);
    lock_maxvel=1;
}

void MainWindow::on_horizontalSlider_max_velocity_sliderReleased()
{
    lock_maxvel=0;
}

void MainWindow::on_horizontalSlider_feed_override_sliderMoved(int position)
{
    nml->setFeedOveride(position*0.01);
    lock_feed_override=1;
}

void MainWindow::on_horizontalSlider_feed_override_sliderReleased()
{
    lock_feed_override=0;
}

void MainWindow::on_horizontalSlider_rapid_override_sliderMoved(int position)
{
    nml->setRapidOverride(position*0.01);
    lock_rapid_override=1;
}

void MainWindow::on_horizontalSlider_rapid_override_sliderReleased()
{
    lock_rapid_override=0;
}

void MainWindow::on_horizontalSlider_spindle_override_sliderMoved(int position)
{
    nml->setSpindleOverride(0,position*0.01);
    lock_spindle_override=1;
}

void MainWindow::on_horizontalSlider_spindle_override_sliderReleased()
{
    lock_spindle_override=0;
}

int MainWindow::get_jog_axis(){
    int jog_axis=0;
    if(ui->radioButton_x->isChecked()){
        jog_axis=0;
    }
    if(ui->radioButton_y->isChecked()){
        jog_axis=1;
    }
    if(ui->radioButton_z->isChecked()){
        jog_axis=2;
    }
    return jog_axis;
}

void MainWindow::on_pushButton_jog_min_pressed()
{
    nml->jog(get_jog_axis(),-ui->horizontalSlider_jog_speed->value()/60);
}

void MainWindow::on_pushButton_jog_min_released()
{
    nml->jog_stop(get_jog_axis());
}

void MainWindow::on_pushButton_jog_plus_pressed()
{
    nml->jog(get_jog_axis(),ui->horizontalSlider_jog_speed->value()/60);
}

void MainWindow::on_pushButton_jog_plus_released()
{
    nml->jog_stop(get_jog_axis());
}

void MainWindow::on_checkBox_mist_clicked(bool checked)
{
    if(checked){
        nml->mist_on();
    } else {
        nml->mist_off();
    }
}

void MainWindow::on_checkBox_flood_clicked(bool checked)
{
    if(checked){
        nml->flood_on();
    } else {
        nml->flood_off();
    }
}

void MainWindow::on_toolButton_spindle_stop_pressed()
{
    nml->spindle_off(0);
}

void MainWindow::on_toolButton_spindle_ccw_pressed()
{
    nml->spindle_on(0,-ui->spinBox_spindlespeed->value()*60);
}

void MainWindow::on_toolButton_spindle_cw_pressed()
{
    nml->spindle_on(0,ui->spinBox_spindlespeed->value()*60);
}

void MainWindow::on_pushButton_spindle_plus_pressed()
{
    ui->spinBox_spindlespeed->setValue(ui->spinBox_spindlespeed->value()+1);

    if(nml->theStatus.spindle_speed<0){
        nml->spindle_on(0,-ui->spinBox_spindlespeed->value()*60);
    }
    if(nml->theStatus.spindle_speed>0){
        nml->spindle_on(0,ui->spinBox_spindlespeed->value()*60);
    }
}

void MainWindow::on_pushButton_spindle_min_pressed()
{
    ui->spinBox_spindlespeed->setValue(ui->spinBox_spindlespeed->value()-1);

    if(nml->theStatus.spindle_speed<0){
        nml->spindle_on(0,-ui->spinBox_spindlespeed->value()*60);
    }
    if(nml->theStatus.spindle_speed>0){
        nml->spindle_on(0,ui->spinBox_spindlespeed->value()*60);
    }
}

void MainWindow::on_toolButton_mdi_command_exec_pressed()
{
    QString mdi_text=ui->lineEdit_mdi_command->text();
    nml->mdi(mdi_text.toStdString());
    nml->mdi_execute();

    ui->plainTextEdit_mdi_history->appendPlainText(mdi_text);
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
    if(index==0){
        nml->mode_manual();
    }
    if(index==1){
        nml->mode_mdi();
    }
}

void MainWindow::on_pushButton_test_pressed()
{

}
