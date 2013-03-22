#ifndef FILTERSWINDOW_H
#define FILTERSWINDOW_H

#include <QMainWindow>

namespace Ui {
    class FiltersWindow;
}

class GuiController;

class FiltersWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit FiltersWindow(GuiController& controller, QWidget *parent = 0);
    ~FiltersWindow();

  private:
    void updateDepthSlider();
    void updateAmplitudeSlider();

private:
    Ui::FiltersWindow *ui;
    GuiController& m_controller;

private slots:
    void on_removeSmallStructuresBox_toggled(bool checked);
    void on_fillSmallHolesCheckBox_toggled(bool checked);
    void on_maxDepthSlider_valueChanged(int value);
    void on_minDepthSlider_valueChanged(int value);
    void on_maxAmplitudeSlider_valueChanged(int value);
    void on_minAmplitudeSlider_valueChanged(int value);
    void on_unstableCheckBox_toggled(bool checked);
    void on_normalsCheckBox_toggled(bool checked);
    void on_medianCheckBox_toggled(bool checked);
    void on_amplitudeCheckBox_toggled(bool checked);
    void on_edgesCheckBox_toggled(bool checked);
    void on_depthThresholdCheckBox_toggled(bool checked);
    void on_kinectTiltSlider_valueChanged(int value);
};

#endif // FILTERSWINDOW_H
