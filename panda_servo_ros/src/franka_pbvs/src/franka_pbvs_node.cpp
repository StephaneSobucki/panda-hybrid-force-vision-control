#include <iostream>
#include <deque>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <franka_pbvs/PandaMsg.h>

#include <visp3/gui/vpPlot.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoData.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <dc1394/dc1394.h>
#include <dc1394/vendor/avt.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if defined(VISP_HAVE_DC1394) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
  (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_FRANKA)

float vx_fcl, vy_fcl, vz_fcl, wx_fcl, wy_fcl, wz_fcl;
vpHomogeneousMatrix T_pandaEE;
int control_law = 1;
//function: callBack_fcl_vel
//	@returns nothing: Triggered every spin. Update velocity coming from comanipulation / teleoperation control
void callBack_fcl_vel(const geometry_msgs::Twist data) {
	vx_fcl = data.linear.x;
	vy_fcl = data.linear.y;
	vz_fcl = data.linear.z;
	wx_fcl = data.angular.x;
	wy_fcl = data.angular.y;
	wz_fcl = data.angular.z;
}
void callBack_pose(const franka_pbvs::PandaMsg& data){
 		
    	T_pandaEE[0][0] = data.ktransform[0];
        T_pandaEE[0][1] = data.ktransform[4];
        T_pandaEE[0][2] = data.ktransform[8];
        T_pandaEE[0][3] = data.ktransform[12];
        T_pandaEE[1][0] = data.ktransform[1];
        T_pandaEE[1][1] = data.ktransform[5];
        T_pandaEE[1][2] = data.ktransform[9];
        T_pandaEE[1][3] = data.ktransform[13];
        T_pandaEE[2][0] = data.ktransform[2];
        T_pandaEE[2][1] = data.ktransform[6];
        T_pandaEE[2][2] = data.ktransform[10];
        T_pandaEE[2][3] = data.ktransform[14];
        //std::cout<<T_pandaEE<<std::endl;
    
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "franka_pbvs_node");
	ros::NodeHandle nh("~");

	std::string s;
	control_law = 1;
	if (nh.getParam("control_law", s)) {
		if (s.compare("pbvs") == 0) {
			control_law = 1;
			ROS_INFO("Using PBVS");
		}
		else if (s.compare("hybrid") == 0) {
			control_law = 2;
			ROS_INFO("Using hybrid parallel Force/Vision control law");
		}
		else if (s.compare("external") == 0) {
			control_law = 3;
			ROS_INFO("Using external hybrid Force/Vision control law");
		}
		else if (s.compare("hybrid_targetlock") == 0) {
			control_law = 4;
			ROS_INFO("Using external hybrid Force/Vision control with target locking");
		}
		else if (s.compare("fcl") == 0) {
			control_law = 5;
			ROS_INFO("Using teleoperation / comanipulation");
		}
		else {
			ROS_INFO("Unknown input ! Available inputs are: pbvs, hybrid, external, fcl");
		}
	}
	else {
		ROS_INFO("Using PBVS as default control law");
	}
	ros::Publisher  command_robot = nh.advertise<geometry_msgs::Twist>("/target_vel", 25);
	ros::Subscriber fcl_vel       = nh.subscribe<geometry_msgs::Twist>("/fcl_vel", 1, &callBack_fcl_vel);
    ros::Subscriber pose          = nh.subscribe("/franka_state_controller/Pose", 1, &callBack_pose);


	double opt_tagSize = 0.019;//0.019;//tag side size in meters
	double opt_plot = false;
	bool display_tag = true;
	int opt_quad_decimate = 0.5;

	//Error threshold
	double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad(0.5);

	try {
		dc1394camera_t* camera;
		dc1394camera_list_t* list;
		dc1394_t* dc;
		dc = dc1394_new();

		//Create a list of available cameras
		dc1394_camera_enumerate(dc, &list);

		//Select the first camera
		camera = dc1394_camera_new(dc, list->ids[0].guid);

		dc1394feature_modes_t gain_modes;
		dc1394feature_modes_t shutter_modes;

		//Get the different modes for gain 
		dc1394_feature_get_modes(camera, DC1394_FEATURE_GAIN, &gain_modes);
		//Get the different modes for shutter
		dc1394_feature_get_modes(camera, DC1394_FEATURE_SHUTTER, &shutter_modes);

		//Mode 0 : Manual, Mode 1 : Auto, Mode 2: Auto one-shot (for gain & shutter)

		//Set the shutter mode to auto
		dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN, gain_modes.modes[1]);

		//Set the gain mode to auto
		dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, shutter_modes.modes[1]);

		bool reset = false;
		vp1394TwoGrabber g(reset);
		vpImage<unsigned char> I;// Create a gray level image container
		//! [vp1394TwoGrabber open]
		g.open(I);

		int width = I.getWidth();
		int height = I.getHeight();

		//Open files for recording servoing positioning errors and velocities
		std::ofstream err_csv;
		err_csv.open("/home/panda/git/panda-hybrid-force-vision-control/servo_data/err.csv");

		std::ofstream vc_csv;
		vc_csv.open("/home/panda/git/panda-hybrid-force-vision-control/servo_data/vc.csv");

                std::ofstream cMo_csv;
		cMo_csv.open("/home/panda/git/panda-hybrid-force-vision-control/servo_data/cMo_csv.csv");
                
                std::ofstream time_csv;
                time_csv.open("/home/panda/git/panda-hybrid-force-vision-control/servo_data/time_csv.csv");
                
                std::ofstream transform_csv;
                transform_csv.open("/home/panda/git/panda-hybrid-force-vision-control/servo_data/transform_csv.csv");

		// Create a camera parameter container
		vpCameraParameters cam;

		//Set camera intrinsic parameters
		double px = 947.71512153725257; 
		double py = 947.28113653925107;
		double u0 = 332.34025661579813;
		double v0 = 228.79043735563889;
		double kud = -0.25791273647851543;
		double kdu = 0.26521071246235034;

		//Set camera intrinsics (perspective model with distorsion)
		cam = vpCameraParameters(px, py, u0, v0, kud, kdu);

#ifdef VISP_HAVE_X11
		vpDisplayX d(I);
#elif defined(VISP_HAVE_OPENCV)
		vpDisplayOpenCV d(I);
#else
		std::cout << "No image viewer is available..." << std::endl;
#endif

		//Apriltag arguments
		vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
		vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::LAGRANGE_VIRTUAL_VS;
		vpDetectorAprilTag detector(tagFamily);

		//Detector instantiation
		detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
		detector.setDisplayTag(display_tag);
		detector.setAprilTagQuadDecimate(opt_quad_decimate);

		//vpHomogeneousMatrix Td_pandaEE(vpTranslationVector(0.44424,0.20293, 0.23114),
                                 //vpRotationMatrix({ 0.406579928, 0.8857390621, 0.2238807038, 0.9135458457, -0.3916419434, -0.1096020297, -0.00939772502, 0.2490872722, -0.1096020297}));

		// Define the required homogeneous transform matrix

		vpHomogeneousMatrix cdMc, cMo, oMo;
		vpHomogeneousMatrix cdMo(vpTranslationVector(0.00459, -0.001481, 0.0638), //(0, 0.0052, 0.07) desired pose for experiment
			vpRotationMatrix({ -1, 0, 0, 0, 1, 0, 0, 0, -1 }));// Apriltag pose in the desired camera frame 

		vpTranslationVector cd_t_c = cdMc.getTranslationVector();
		vpThetaUVector ThetaU = cdMc.getThetaUVector();

		//Spatial motion transform matrix from visp camera frame to real camera frame
		vpMatrix vispVc(6, 6);

		vispVc[0][1] = 1;
		vispVc[1][0] = 1;
		vispVc[2][2] = -1;
		vispVc[3][4] = 1;
		vispVc[4][3] = 1;
		vispVc[5][5] = -1;

		cdMc = cdMo * cMo.inverse();
                
		//Choice of features for our visual servoing task
		vpFeatureTranslation t(vpFeatureTranslation::cdMc);
		vpFeatureThetaU tu(vpFeatureThetaU::cdRc);

		//Queue to keep track of translational error in Oxy plane
		std::deque<double> txy_queue;
		int queue_size = 5;

                //Track servoing time
                double servo_time = 0;

		vpServo task;
		task.addFeature(t);
		task.addFeature(tu);

		//Set servo type (eye-in-hand, velocities in operational space)
		task.setServo(vpServo::EYEINHAND_CAMERA);

		//Compute interaction matrix at each step
		task.setInteractionMatrixType(vpServo::CURRENT);

		//Servoing gain
		double lambda;
                //double lambda_t = 2;

		//Adaptive gain for angular velocities in Hybrid control laws
		vpAdaptiveGain lambda_tu_ada(0.6, 0.2, 30);

                //vpAdaptiveGain lambda_t_ada(7,2,30);

		bool final_quit = false;
		bool has_converged = false;
		bool send_velocities = false;
		bool servo_started = false;
		bool target_lock;


		while (!has_converged && !final_quit) {
			double t_start = vpTime::measureTimeMs();

			g.acquire(I);
			vpDisplay::display(I);

			if (!servo_started) {
				if (control_law != 1) {
					bool vx_zero = vx_fcl == 0.0f;
					bool vy_zero = vy_fcl == 0.0f;
					bool vz_zero = vz_fcl == 0.0f;

					servo_started = !(vx_zero && vy_zero && vz_zero);
				}
				else {
					servo_started = true;
				}
			}

			std::vector<vpHomogeneousMatrix> cMo_vec;
			detector.detect(I, opt_tagSize, cam, cMo_vec);

			std::stringstream ss;
			ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
			vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

			vpColVector v_c(6);

			// Only one tag is detected
			if (cMo_vec.size() == 1) {

				cMo = cMo_vec[0];
                                
				vpHomogeneousMatrix ds;

				//External Hybrid Force/Vision
				if (control_law == 3 || control_law == 4) {
					ds = vpHomogeneousMatrix(vpTranslationVector(vx_fcl, vy_fcl, vz_fcl), vpRotationMatrix({ 0,-1,0,-1,0,0,0,0,1 }));
				}

				static bool first_time = true;
				if (first_time) {
					// Introduce security wrt tag positionning in order to avoid PI rotation
					std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
					v_oMo[0] << 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1;

					v_oMo[1] << -1, 0, 0, 0,
						0, -1, 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1;
					for (size_t i = 0; i < 2; i++) {
						v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
					}
					if (std::fabs(v_cdMc[0].getThetaUVector()[2]) < std::fabs(v_cdMc[1].getThetaUVector()[2])) {
						oMo = v_oMo[0];
					}
					else {
						std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
						oMo = v_oMo[1];   // Introduce PI rotation
						std::cout << oMo << std::endl;
					}
				}

				// Display desired and current pose features
				vpDisplay::displayFrame(I, cdMo, cam, opt_tagSize / 1.5, vpColor::none, 3);
				vpDisplay::displayFrame(I, cMo, cam, opt_tagSize / 2, vpColor::none, 3);

				// Update visual features
				cdMc = cdMo * oMo * cMo.inverse();

				cd_t_c = cdMc.getTranslationVector();
				ThetaU = cdMc.getThetaUVector();
				double error_tr = sqrt(cd_t_c.sumSquare());
				double error_tu = vpMath::deg(sqrt(ThetaU.sumSquare()));
				double error_tr_x_y = sqrt(cd_t_c[0] * cd_t_c[0] + cd_t_c[1] * cd_t_c[1]);

				//Update the translation error vector in Oxy plane. It's a five element FIFO array. 
				if (txy_queue.size() < queue_size) {
					txy_queue.push_back(error_tr_x_y);
				}
				else {
					txy_queue.pop_front();
					txy_queue.push_back(error_tr_x_y);
				}

				switch (control_law) {
					//PBVS
				case 1:
					t.buildFrom(cdMc);
					tu.buildFrom(cdMc);
					break;
				case 2:
					// Hybrid parallel
					tu.buildFrom(cdMc);
					break;
					//External Hybrid Force/Vision
				case 3:
					t.buildFrom(ds);
					tu.buildFrom(cdMc);
					break;
					// External hybrid with target locking 	 
				case 4:
					{
                                                t.buildFrom(ds);
						//Angle calculation for target locking
						double w_x = atan2(cMo[1][3], cMo[2][3]);
						double w_y = -atan2(cMo[0][3], cMo[2][3]);

						vpHomogeneousMatrix cdMc_lock(vpTranslationVector(0.00143, -0.00148, 0.0638), // 0.21m along camera z axis
							vpThetaUVector(w_x, w_y, ThetaU[2]));

						//We use the mean translation error in Oxy plane, in order to account for noisy pose estimation
						//TO-DO: implement kalman filter for pose estimation
						double mean_err = 0;
						for (std::deque<double>::const_iterator it(txy_queue.begin()); it != txy_queue.end(); it++) {
							mean_err += *it;
						}
						mean_err /= txy_queue.size();
						if (mean_err < 2*opt_tagSize) {
							ss.str("");
							ss << "Mode: VS";
							vpDisplay::displayText(I, 60, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::green);
							tu.buildFrom(cdMc);
							target_lock = false;
						}
						else {
							ss.str("");
							ss << "Mode: TARGET LOCK";
							vpDisplay::displayText(I, 60, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::green);
							tu.buildFrom(cdMc_lock);
							target_lock = true;
						}
						break;
					}
				default: 
					break;
				}

				//For hybrid control laws
				//Compute adaptive gain for angular velocities with respect to the infinity norm of the translation error vector
				if (control_law == 1) {
					lambda = 0.08;
				}
				else {
					lambda = lambda_tu_ada.value(vpColVector(cdMc.getTranslationVector()).infinityNorm());
                                        //lambda_t = lambda_t_ada.value(vpColVector(cdMc.getTranslationVector()).infinityNorm());
				}

				v_c = task.computeControlLaw();
				v_c = vispVc * v_c;

				//Display positionning error on the ViSP image window
				ss.str("");
				ss << "error_t: " << error_tr;
				vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);
				ss.str("");
				ss << "error_tu: " << error_tu;
				vpDisplay::displayText(I, 40, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

				//Check if the positioning error is below the convergence criteria 
				if (error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu) {
					has_converged = true;
					std::cout << "Servo task has converged" << std::endl;
					vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
					v_c = 0;
				}

				if (first_time) {
					first_time = false;
				}
			}// end if (cMo_vec.size() == 1)
			else {
				v_c = 0;
			}

			if (!send_velocities) {
				v_c = 0;
			}

                        double loop_time; 
                        loop_time = vpTime::measureTimeMs() - t_start;
                        if(servo_started){
				servo_time += loop_time;
                        }
			//Display loop duration on the ViSP image window
			ss.str("");
			ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
			vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
			vpDisplay::flush(I);

			//Left click on the ViSP image window to start/stop sending velocities
			vpMouseButton::vpMouseButtonType button;
			if (vpDisplay::getClick(I, button, false)) {
				switch (button) {
				case vpMouseButton::button1:
					send_velocities = !send_velocities;
					break;

				case vpMouseButton::button3:
					//Right click to quit servoing
					final_quit = true;
					err_csv.close();
					vc_csv.close();
                                        cMo_csv.close();
                                        time_csv.close();
                                        transform_csv.close();
					v_c = 0;
					break;

				default:
					break;
				}
			}

			//Message to send velocity screw
			geometry_msgs::Twist twist_val;

			//For hybrid parallel Force/Vision and comanipulation / teleoperation
			if (control_law == 2 || control_law == 5) {
				twist_val.linear.x = vx_fcl;
				twist_val.linear.y = vy_fcl;
				twist_val.linear.z = vz_fcl;
			}
			else {
				twist_val.linear.x = v_c[0];
				twist_val.linear.y = v_c[1];
				twist_val.linear.z = v_c[2];
			}

			//External hybrid force/vision with target locking
			if (control_law == 4) {
				//In target locking task
				if (target_lock) {
					twist_val.angular.x = 0.5*v_c[3];
					twist_val.angular.y = 0.5*v_c[4];
				}
				else {
					twist_val.angular.x = lambda * v_c[3];
					twist_val.angular.y = lambda * v_c[4];
				}
				twist_val.angular.z = lambda * v_c[5];
			}
			//PBVS
			else if (control_law == 1) {
				twist_val.angular.x = lambda * v_c[3];
				twist_val.angular.y = lambda * v_c[4];
				twist_val.angular.z = lambda * v_c[5];
			}
			//Comanipulation / teleoperation
			else if (control_law == 5) {
				twist_val.angular.x = wx_fcl;
				twist_val.angular.y = wy_fcl;
				twist_val.angular.z = wz_fcl;
			}
			//Hybrid parallel force/vision & external hybrid force/vision
			else {
				twist_val.angular.x = lambda * v_c[3];
				twist_val.angular.y = lambda * v_c[4];
				twist_val.angular.z = lambda * v_c[5];
			}

			if (servo_started) {
				vc_csv << twist_val.linear.x << "," << twist_val.linear.y << "," << twist_val.linear.z << "," << twist_val.angular.x << "," << twist_val.angular.y << "," << twist_val.angular.z << "\n";
                                time_csv << servo_time << "\n";
				cMo_csv << cMo[0][0] << "," << cMo[0][1] << "," << cMo[0][2] << "," << cMo[0][3] << ","
                                        << cMo[1][0] << "," << cMo[1][1] << "," << cMo[1][2] << "," << cMo[1][3] << ","
                                        << cMo[2][0] << "," << cMo[2][1] << "," << cMo[2][2] << "," << cMo[2][3] << "," 
                                        << cMo[3][0] << "," << cMo[3][1] << "," << cMo[3][2] << "," << cMo[3][3] << "\n";

				err_csv << cd_t_c[0] << "," << cd_t_c[1] << "," << cd_t_c[2] << "," << ThetaU[0] << "," << ThetaU[1] << "," << ThetaU[2] << "\n";

                                transform_csv << T_pandaEE[0][0] << "," << T_pandaEE[0][1] << "," << T_pandaEE[0][2] << "," << T_pandaEE[0][3] << ","
                                              << T_pandaEE[1][0] << "," << T_pandaEE[1][1] << "," << T_pandaEE[1][2] << "," << T_pandaEE[1][3] << ","
					      << T_pandaEE[2][0] << "," << T_pandaEE[2][1] << "," << T_pandaEE[2][2] << "," << T_pandaEE[2][3] << ","
					      << T_pandaEE[3][0] << "," << T_pandaEE[3][1] << "," << T_pandaEE[3][2] << "," << T_pandaEE[3][3] << "," << "\n";



				command_robot.publish(twist_val);
			}

			ros::spinOnce();
		}
		std::cout << "Stop the robot " << std::endl;

		task.kill();

		if (!final_quit) {
			while (!final_quit) {
				g.acquire(I);
				vpDisplay::display(I);

				vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
				vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpColor::red);

				if (vpDisplay::getClick(I, false)) {
					final_quit = true;
				}

				vpDisplay::flush(I);
			}
		}
	}
	catch (const vpException& e) {
		std::cout << "ViSP exception: " << e.what() << std::endl;

		geometry_msgs::Twist twist_val;

		command_robot.publish(twist_val);

		return EXIT_FAILURE;
	}

	return 0;
}
#else
int main()
{
#if !defined(VISP_HAVE_libdc1394)
	std::cout << "Install libdc1394" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
	std::cout << "Build ViSP with c++11 or higher compiler flag (cmake -DUSE_CXX_STANDARD=11)." << std::endl;
#endif
	return 0;
}
#endif
