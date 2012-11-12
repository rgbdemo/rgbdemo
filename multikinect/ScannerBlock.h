#ifndef SCANNERBLOCK_H
#define SCANNERBLOCK_H

#include <ntk/camera/rgbd_grabber.h>
#include <ntk/thread/event.h>
#include <ntk/thread/utils.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/mesh_generator.h>

#include <set>

namespace ntk
{
    class RGBDFrameRecorder;
}

struct FrameVector : public ntk::EventData
{
    TYPEDEF_THIS(FrameVector)

    CLONABLE_EVENT_DATA

    std::vector<ntk::RGBDImagePtr> images;
};
ntk_ptr_typedefs(FrameVector)

struct FrameVectorVector : public ntk::EventData
{
    TYPEDEF_THIS(FrameVectorVector)

    CLONABLE_EVENT_DATA

    std::vector<FrameVectorPtr> frames;
};
ntk_ptr_typedefs(FrameVectorVector)

struct MeshVector : public ntk::EventData
{
    TYPEDEF_THIS(MeshVector)

    CLONABLE_EVENT_DATA

    std::vector<std::string> camera_serials;
    std::vector<ntk::MeshPtr> meshes;
};
ntk_ptr_typedefs(MeshVector)

struct CalibrationParameters : public ntk::EventData
{
    TYPEDEF_THIS(CalibrationParameters)

    CLONABLE_EVENT_DATA

    std::string camera_serial;
    const ntk::RGBDCalibration* calibration;
    cv::Vec3f new_t;
    cv::Vec3f new_r;
};
ntk_ptr_typedefs(CalibrationParameters)

class ScannerBlock : public ntk::EventProcessingBlockInOwnThread
{
public:
    ScannerBlock(Name name)
    : ntk::EventProcessingBlockInOwnThread(name)
    , m_connected(true)
    {}

public:
    void setConnected(bool connected) { m_connected = connected; }
    bool isConnected() const { return m_connected; }

public:
    virtual void newEvent(ntk::EventBroadcaster* sender = 0, ntk::EventDataPtr data = ntk::EventDataPtr())
    {
        if (m_connected)
            ntk::EventProcessingBlockInOwnThread::newEvent(sender, data);
    }

private:
    bool m_connected;
};

class RecorderBlock : public ScannerBlock
{
public:
    RecorderBlock(Name name);

public:
    void setOutputDirectoryPrefix(const std::string& dir);

protected:
    virtual void run();

private:
    ntk::RGBDFrameRecorder* m_recorder;
    QMutex m_recorder_mutex;
};

class RGBDProcessorBlock : public ScannerBlock
{
public:
    RGBDProcessorBlock(Name name)
    : ScannerBlock(name)
    {}

public:
    void setMinDepth(float depth) { m_processor.setMinDepth(depth); }
    void setMaxDepth(float depth) { m_processor.setMaxDepth(depth); }
    float minDepth() const { return m_processor.minDepth(); }
    float maxDepth() const { return m_processor.maxDepth(); }

    void setProcessorFilterFlag(ntk::RGBDProcessorFlags::Flag flag, bool enabled);
    ntk::RGBDProcessorFlags getProcessorFilterFlags() const { return m_processor.getFilterFlags(); }

protected:
    virtual void run();

private:
    ntk::OpenniRGBDProcessor m_processor;
    QMutex m_processor_mutex;
};

class FrameSynchronizerBlock : public ScannerBlock
{
public:
    FrameSynchronizerBlock(Name name)
    : ScannerBlock(name),
      m_wait_until_only_one_has_depth(false)
    {}

public:
    /*! Add a device that must be waited for. */
    void addDevice(const std::string& serial) { m_devices_serial_to_wait.insert(serial); }

    void setWaitUntilOnlyOneFrameHasDepthMode(bool enable);

protected:
    virtual void run();

private:
    bool otherImagesHaveNoDepth(const std::string& serial) const;

private:
    bool m_wait_until_only_one_has_depth;
    mutable QMutex mutex;
    std::set<std::string> m_devices_serial_to_wait;
    typedef std::map<std::string, ntk::RGBDImagePtr> image_map_type;
    image_map_type m_updated_images;
    typedef std::map<std::string, uint64> images_without_depth_map_type;
    images_without_depth_map_type m_images_without_depth;
    std::string m_last_added_image;
};

class MeshGeneratorBlock : public ScannerBlock
{
public:
    MeshGeneratorBlock(Name name);

public:
    void setMeshType(ntk::MeshGenerator::MeshType type);
    void setMeshUseColor(bool use_it);
    void setMeshResolutionFactor(double value);

protected:
    virtual void run();

private:
    QMutex m_generator_mutex;
    ntk::MeshGenerator m_mesh_generator;
};

class CalibratorBlock : public ScannerBlock
{
public:
    class Controller
    {
    public:
        virtual void onCameraExtrinsicsChanged();
    };

public:
    enum Algorithm { ICP, Chessboard };

public:
    static double a4SquareSize () { return 0.025; }
    static double a3SquareSize () { return 0.034; }

public:
    CalibratorBlock(Name name)
    : ScannerBlock(name)
    , m_controller(0)
    , m_algo(ICP)
    , m_pattern_size(a4SquareSize())
    , m_pattern_width(10)
    , m_pattern_height(7)
    {}

public:
    void plugController(Controller* controller) { m_controller = controller; }
    void setCalibrationAlgorithm(Algorithm algo);
    void setCalibrationPattern(float square_size, int num_corners_x, int num_corners_y);

protected:
    void calibrateWithICP(FrameVectorVectorConstPtr frames);
    void calibrateWithChessboard(FrameVectorVectorConstPtr frames);

protected:
    virtual void run();

private:
    Controller* m_controller;
    RecursiveQMutex m_calibrator_mutex;
    Algorithm m_algo;
    float m_pattern_size;
    int m_pattern_width;
    int m_pattern_height;
    std::vector<FrameVectorConstPtr> m_checkerboard_frames;
};

#endif // SCANNERBLOCK_H
