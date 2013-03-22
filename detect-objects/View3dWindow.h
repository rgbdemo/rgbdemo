#ifndef VIEW3DWINDOW_H
#define VIEW3DWINDOW_H

#include <QMainWindow>

namespace Ui {
    class View3DWindow;
}

class GuiController;

class View3DWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit View3DWindow(GuiController& controller, QWidget *parent = 0);
    ~View3DWindow();

private:
    Ui::View3DWindow *ui;
    GuiController& m_controller;

private slots:
    void on_saveMeshPushButton_clicked();
    void on_trianglePushButton_clicked();
    void on_surfelsPushButton_clicked();
    void on_pointCloudPushButton_clicked();
    void on_findObjectsCheckBox_toggled(bool checked);
    void on_showPlaneCheckBox_toggled(bool checked);
    void on_resolutionFactorSpinBox_valueChanged(double );
    void on_colorMappingCheckBox_toggled(bool checked);
    void on_resetCamera_clicked();

    friend class GuiController;
};

#endif // VIEW3DWINDOW_H
