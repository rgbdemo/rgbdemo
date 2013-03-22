#ifndef OBJECT_OBJECT_WINDOW_H
#define OBJECT_OBJECT_WINDOW_H

#include <ntk/core.h>

#include <QMainWindow>

#include <ntk/geometry/relative_pose_estimator.h>
#include <ntk/detection/object/object_finder.h>

namespace Ui {
    class ObjectWindow;
}

class GuiController;

class ObjectWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ObjectWindow(GuiController& controller, QWidget *parent = 0);
    ~ObjectWindow();

public:
    void setDatabasePath(const std::string& path);
    void registerNewModel(const ntk::RGBDImage& image);
    void processNewFrame(const ntk::RGBDImage& image);
    void updateView();

private slots:
    void on_resetCamera_clicked();
    void on_detectButton_clicked();

    void on_databasePathLineEdit_editingFinished();

    void on_browseButton_clicked();

    void on_showObjectCheckBox_toggled(bool checked);

private:
    void initializeDetector();
    void resetDetector();

private:
    Ui::ObjectWindow *ui;
    GuiController& m_controller;
    std::string m_database_path;
    ntk::ObjectFinderPtr m_object_finder;
    ntk::Mesh m_table_mesh;
    ntk::Mesh m_object_mesh;
    ntk::Mesh m_scene_mesh;
    int m_current_model_index;
    bool m_show_detection;

private:
    friend class GuiController;
    RecursiveQMutex m_lock;
};

#endif // OBJECT_OBJECT_WINDOW_H
