/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#ifndef MULTIKINECTSCANNER_H
#define MULTIKINECTSCANNER_H

#include "ScannerBlock.h"

#include <ntk/camera/rgbd_grabber.h>
#include <ntk/thread/event.h>
#include <ntk/thread/utils.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/mesh_generator.h>

#include <QThread>

#include <set>

class MultiKinectController;

class MultiKinectScanner : public ScannerBlock
{
public:
    struct DeviceInfo
    {
        DeviceInfo() : calibration(0) {}

        std::string serial;
        const ntk::RGBDCalibration* calibration;
    };

public:
    MultiKinectScanner();

    bool initialize();

public: // control
    void plugController(MultiKinectController* controller) { m_controller = controller; }

    void setIRMode(bool enable);
    void setGrabberSynchronous(bool sync);
    bool areGrabbersSynchronous() const;

    bool canGenerateMeshes() const { return true; } // FIXME

public:
    void addGrabber(ntk::RGBDGrabber* grabber);

public:
    RGBDProcessorBlock& processorBlock() { return m_processor_block; }
    const RGBDProcessorBlock& processorBlock() const { return m_processor_block; }

    RecorderBlock& recorderBlock() { return m_recorder_block; }
    const RecorderBlock& recorderBlock() const { return m_recorder_block; }

    MeshGeneratorBlock& meshGeneratorBlock() { return m_mesh_generator_block; }
    const MeshGeneratorBlock& meshGeneratorBlock() const { return m_mesh_generator_block; }

    CalibratorBlock& calibratorBlock() { return m_calibrator_block; }
    const CalibratorBlock& calibratorBlock() const { return m_calibrator_block; }

    void setPaused(bool paused);
    bool isPaused() const { return m_paused; }

    void processOneFrame();

    int numDevices() const { return m_devices.size(); }
    DeviceInfo getDeviceInfo(int device) { return m_devices[device]; }

    void saveGrabbersCalibration(const std::string &prefix);
    void calibrateCameras(FrameVectorVectorPtr frames);
    void updateCameraCalibration(CalibrationParametersPtr params);
    FrameVectorPtr lastProcessedFrameVector() const { return m_last_processed_frame_vector; }

    void setWaitUntilOnlyOneFrameHasDepthMode(bool enable);

protected:
    void processLastImageFromGrabber(ntk::RGBDGrabber& grabber);
    void triggerGrabbers();

protected:
    virtual void run();

private:
    RecorderBlock m_recorder_block;
    RGBDProcessorBlock m_processor_block;
    FrameSynchronizerBlock m_frame_synchronizer_block;
    MeshGeneratorBlock m_mesh_generator_block;
    CalibratorBlock m_calibrator_block;

private:
    std::set<ntk::RGBDGrabber*> m_grabbers;
    std::vector<DeviceInfo> m_devices;
    FrameVectorPtr m_last_processed_frame_vector;

private: // control
    MultiKinectController* m_controller;
    bool m_paused;
    int m_process_one_frame;
};

class MultiKinectController
{
public:
    MultiKinectController(MultiKinectScanner* scanner) : m_scanner(scanner) {}

public: // actions
    void quit();

public: // modifiers
    MultiKinectScanner& scanner() { return *m_scanner; }
    const MultiKinectScanner& scanner() const { return *m_scanner; }

public: // callbacks
    virtual void onScannerInitialized() {}
    virtual void onNewImage(ntk::RGBDImageConstPtr image) {}
    virtual void onNewSynchronizedImages(const std::vector<ntk::RGBDImagePtr>& images) {}
    virtual void onNewSynchronizedMeshes(MeshVectorPtr meshes) {}
    virtual void onCameraExtrinsicsChanged(CalibrationParametersPtr) {}

private:
    MultiKinectScanner* m_scanner;
};

#endif // MULTIKINECTSCANNER_H
