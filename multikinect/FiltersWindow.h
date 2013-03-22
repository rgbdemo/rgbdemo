#ifndef FILTERSWINDOW_H
#define FILTERSWINDOW_H

#include <QMainWindow>

namespace Ui {
    class FiltersWindow;
}

class GuiMultiKinectController;

class FiltersWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit FiltersWindow(GuiMultiKinectController& controller, QWidget *parent = 0);
    ~FiltersWindow();

  private:
    void updateDepthSlider();
#if FIXME
    void updateAmplitudeSlider();
#endif

private:
    Ui::FiltersWindow *ui;
    GuiMultiKinectController& m_controller;

private slots:
    void on_maxDepthSlider_valueChanged(int value);
    void on_minDepthSlider_valueChanged(int value);
    void on_depthThresholdCheckBox_toggled(bool checked);
    void on_medianCheckBox_toggled(bool checked);
    void on_removeSmallStructuresBox_toggled(bool checked);
    void on_fillSmallHolesCheckBox_toggled(bool checked);

#if FIXME    
    void on_maxAmplitudeSlider_valueChanged(int value);
    void on_minAmplitudeSlider_valueChanged(int value);
    void on_unstableCheckBox_toggled(bool checked);
    void on_normalsCheckBox_toggled(bool checked);

    void on_amplitudeCheckBox_toggled(bool checked);
    void on_edgesCheckBox_toggled(bool checked);
    void on_kinectTiltSlider_valueChanged(int value);
#endif
};

#endif // FILTERSWINDOW_H
