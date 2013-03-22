#ifndef RAWIMAGESWINDOW_H
#define RAWIMAGESWINDOW_H

#include <ntk/camera/calibration.h>

#include <QMainWindow>

namespace Ui {
    class RawImagesWindow;
}

class GuiController;

class RawImagesWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit RawImagesWindow(GuiController& controller, QWidget *parent = 0);
    ~RawImagesWindow();

public:
    void update(const ntk::RGBDImage& image);

private:
    Ui::RawImagesWindow *ui;
    GuiController& m_controller;

  public slots:
    void closeEvent(QCloseEvent *event);

private slots:
    void on_actionObject_tracker_toggled(bool);
    void on_actionSave_calibration_parameters_triggered();
    void on_integrationTimeText_returnPressed();
    void on_actionNext_frame_triggered();
    void on_actionPause_toggled(bool );
    void on_syncMode_toggled(bool checked);
    void on_action_GrabFrames_toggled(bool );
    void on_action_Screen_capture_mode_toggled(bool );
    void on_action_Filters_toggled(bool );
    void on_action_3D_View_toggled(bool );
    void on_action_Quit_triggered();
    void on_action_GrabOneFrame_triggered();
    void on_outputDirText_editingFinished();
    void on_actionShow_IR_toggled(bool v);
    void on_actionDual_RGB_IR_mode_toggled(bool v);
    friend class GuiController;
};

#endif // RAWIMAGESWINDOW_H
