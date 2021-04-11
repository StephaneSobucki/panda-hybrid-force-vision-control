//! \example tutorial-franka-acquire-calib-data.cpp
#include <iostream>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <dc1394/dc1394.h>
#include <dc1394/vendor/avt.h>

#if defined(VISP_HAVE_DC1394) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

int main(int argc, char **argv)
{
  try {
    vpImage<unsigned char> I;

	dc1394camera_t *camera;
	dc1394camera_list_t *list;
	dc1394_t *d;
	d = dc1394_new();
	
	dc1394_camera_enumerate(d, &list);
	
	camera = dc1394_camera_new(d,list->ids[0].guid);

	dc1394feature_modes_t gain_modes;
	dc1394feature_modes_t shutter_modes;

	dc1394_feature_get_modes(camera, DC1394_FEATURE_GAIN, &gain_modes);
	
	dc1394_feature_get_modes(camera, DC1394_FEATURE_SHUTTER, &shutter_modes);

	dc1394_feature_set_mode(camera,DC1394_FEATURE_GAIN, gain_modes.modes[1]);

	dc1394_feature_set_mode(camera,DC1394_FEATURE_SHUTTER, shutter_modes.modes[1]);

    bool reset = false;
    vp1394TwoGrabber g(reset);
    g.acquire(I);

    unsigned int width = I.getWidth();
    unsigned int height = I.getHeight();

    std::cout << "Image size: " << width << " x " << height << std::endl;

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

    bool end = false;
    unsigned cpt = 0;
    while (! end) {
      g.acquire(I);

      vpDisplay::display(I);

      vpDisplay::displayText(I, 15, 15, "Left click to acquire data", vpColor::red);
      vpDisplay::displayText(I, 30, 15, "Right click to quit", vpColor::red);
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button1) {
          cpt ++;

          std::stringstream ss_img, ss_pos;
					(cpt < 10)? ss_img<<"image-0" << cpt << ".jpg":ss_img << "image-" << cpt << ".jpg";
					
          std::cout << "Save: " << ss_img.str() << std::endl;
          vpImageIo::write(I, ss_img.str());
        }
        else if (button == vpMouseButton::button3) {
          end = true;
        }
      }
      vpDisplay::flush(I);

    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
#else
int main()
{
#if !(VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::cout << "Build ViSP with c++11 or higher compiler flag (cmake -DUSE_CXX_STANDARD=11)." << std::endl;
#endif
#if !defined(VISP_HAVE_libdc1394)
	std::cout << "Install libdc1394" << std::endl;
#endif
  std::cout << "After installation of the missing 3rd parties, configure ViSP with cmake"
            << " and build ViSP again." << std::endl;
  return 0;
}
#endif
