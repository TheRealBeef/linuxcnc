#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "QFileDialog"
#include <QTimer>
#include <iostream>

//! Make conversion's easy:
#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)

#ifdef Success
#undef Success
#endif

//! libocc.
#include "draw_primitives.h"
#include "OcctQtViewer.h"

//! nml.
#include <config.h>
#include <emc.hh>
#include <emc_nml.hh>

//! cad data from interpreter
#include "common.h"

//! occ overlay, ensure it's a created from qt->add_new->dialog_gui_class->widget.
#include "form.h"

//! gcode editor.
#include "QGCodeEditor.h"

//! Lcnc server, client interface.
#include "nml.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void update();
    int digits(int nr);

private slots:
    void load_cad_data_from_interpreter(std::string filename);
    void load_gcode_text(std::string filename);
    int get_jog_axis();

    void on_toolButton_fit_all_pressed();

    void on_toolButton_view_front_pressed();

    void on_toolButton_view_back_pressed();

    void on_toolButton_view_top_pressed();

    void on_toolButton_view_bottom_pressed();

    void on_toolButton_view_left_pressed();

    void on_toolButton_view_right_pressed();

    void on_toolButton_view_3d_pressed();

    void on_toolButton_zoom_min_pressed();

    void on_toolButton_zoom_plus_pressed();

    void on_toolButton_stop_pressed();

    void on_toolButton_pause_toggled(bool checked);

    void on_toolButton_run_pressed();

    void on_toolButton_file_open_pressed();

    void on_toolButton_reload_pressed();

    void on_toolButton_emergency_pressed();

    void on_toolButton_machine_on_pressed();

    void on_horizontalSlider_max_velocity_sliderMoved(int position);

    void on_horizontalSlider_max_velocity_sliderReleased();

    void on_horizontalSlider_feed_override_sliderMoved(int position);

    void on_horizontalSlider_feed_override_sliderReleased();

    void on_horizontalSlider_rapid_override_sliderMoved(int position);

    void on_horizontalSlider_rapid_override_sliderReleased();

    void on_horizontalSlider_spindle_override_sliderMoved(int position);

    void on_horizontalSlider_spindle_override_sliderReleased();

    void on_pushButton_jog_min_pressed();

    void on_pushButton_jog_min_released();

    void on_pushButton_jog_plus_pressed();

    void on_pushButton_jog_plus_released();

    void on_checkBox_mist_clicked(bool checked);

    void on_checkBox_flood_clicked(bool checked);

    void on_toolButton_spindle_stop_pressed();

    void on_toolButton_spindle_ccw_pressed();

    void on_toolButton_spindle_cw_pressed();

    void on_pushButton_spindle_plus_pressed();

    void on_pushButton_spindle_min_pressed();

    void on_toolButton_mdi_command_exec_pressed();

    void on_tabWidget_currentChanged(int index);

private:
    Ui::MainWindow *ui;

    RCS_STAT_CHANNEL *stat;
    EMC_STAT  *emcStatus;

    OcctQtViewer *occ;

    Form *form = new Form();

    Nml *nml = new Nml();

    QGCodeEditor *editor;
};
#endif
