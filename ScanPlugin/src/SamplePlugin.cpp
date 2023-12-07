#include "SamplePlugin.hpp"
#include <rw/core/macros.hpp>
#include <filesystem>

SamplePlugin::SamplePlugin () :
    RobWorkStudioPlugin (
        "SamplePluginUI",
        QIcon ((std::filesystem::path (__FILE__).parent_path () / "pa_icon.png").c_str ()))
{
    setupUi (this);

    _timer = new QTimer (this);
    connect (_timer, SIGNAL (timeout ()), this, SLOT (on_timerTick ()));

    // now connect stuff from the ui component
    connect (_btn_scan, SIGNAL (pressed ()), this, SLOT (on_btnGetScan ()));
    connect (_spinBox, SIGNAL (valueChanged (int)), this, SLOT (on_spinBoxChanged (int)));

    _cameras25D = {"Scanner25D"};
}

SamplePlugin::~SamplePlugin ()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize ()
{
    log ().info () << "INITALIZE"
                   << "\n";

    getRobWorkStudio ()->stateChangedEvent ().add (
        std::bind (&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Auto load workcell
    std::filesystem::path wc_path (__FILE__);
    wc_path = wc_path.parent_path () / "../../WorkCell/Scene.wc.xml";


    WorkCell::Ptr wc = WorkCellLoader::Factory::load (wc_path.string ());

    if (wc.isNull ()) {
        RW_THROW ("Could not load WorkCell from: " << wc_path);
    }
    getRobWorkStudio ()->setWorkCell (wc);
}

void SamplePlugin::open (WorkCell* workcell)
{
    log ().info () << "OPEN"
                   << "\n";
    _wc    = workcell;
    _state = _wc->getDefaultState ();

    log ().info () << workcell->getFilename () << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame ("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "TextureImage", _textureRender, textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame ("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "BackgroundImage", _bgRender, bgFrame);
        }

        Frame* cameraFrame25D = _wc->findFrame (_cameras25D[0]);
        if (cameraFrame25D != NULL) {
            if (cameraFrame25D->getPropertyMap ().has ("Scanner25D")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam =
                    cameraFrame25D->getPropertyMap ().get< std::string > ("Scanner25D");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber25D          = new GLFrameGrabber25D (width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio ()->getView ()->getSceneViewer ();
                _framegrabber25D->init (gldrawer);
            }
        }

        _device = _wc->findDevice ("UR-6-85-5-A");
        _step   = -1;
    }
}

void SamplePlugin::close ()
{
    log ().info () << "CLOSE"
                   << "\n";

    // Stop the timer
    _timer->stop ();
    // Remove the texture render
    Frame* textureFrame = _wc->findFrame ("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("TextureImage", textureFrame);
    }
    // Remove the background render
    Frame* bgFrame = _wc->findFrame ("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("BackgroundImage", bgFrame);
    }

    _wc           = NULL;
}

void SamplePlugin::on_btnGetScan (){ 
        get25DImage (0);
}

void SamplePlugin::on_spinBoxChanged(int value){
    log ().info () << "spin value:" << value << "\n";
}

void SamplePlugin::get25DImage (int index)
{
    Frame* duckFrame = _wc->findFrame ("Duck");
    Frame* scannerFrame = _wc->findFrame ("Scanner25D");

    Transform3D<> dToC = Kinematics::frameTframe(scannerFrame, duckFrame, _state);

    std::ofstream file("gt_" + std::to_string(index) + ".txt");
    if ( file.is_open()) {
        file << dToC.e() << std::endl;
        file.close();
    } else {
        std::cerr << "Could not open file!" << std::endl;
    }

    if (_framegrabber25D != NULL) {
        for (size_t i = 0; i < _cameras25D.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame25D = _wc->findFrame (_cameras25D[i]);    // "Camera");
            _framegrabber25D->grab (cameraFrame25D, _state);

            // const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage ());

            std::ofstream output ("scene_" + std::to_string(index) + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth () << "\n";
            output << "HEIGHT " << img->getHeight () << "\n";
            output << "POINTS " << img->getData ().size () << "\n";
            output << "DATA ascii\n";
            for (const auto& p_tmp : img->getData ()) {
                rw::math::Vector3D< float > p = p_tmp;
                output << p (0) << " " << p (1) << " " << p (2) << "\n";
            }
            output.close ();
        }
    }
}

void SamplePlugin::on_timerTick ()
{
    if (0 <= _step && (size_t) _step < _path.size ()) {
        _device->setQ (_path.at (_step), _state);
        getRobWorkStudio ()->setState (_state);
        _step++;
    }
}


void SamplePlugin::stateChangedListener (const State& state)
{
    _state = state;
}
