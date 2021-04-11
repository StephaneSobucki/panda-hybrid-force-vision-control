#include <iostream>
#include <cmath>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpDisplayOpenCV.h>

#include "b0RemoteApi.h"

b0RemoteApi* cl = NULL;

int vision_sensor_handle;

bool do_next_step = true;
bool ready_for_servoing = false;
bool has_converged = false;

//Camera resolution
double width = 640;
double height = 480;

vpImage<unsigned char> I;
vpCameraParameters cam;

//Apriltag detector
vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
vpDetectorAprilTag detector(tagFamily);

//Apriltag arguments
double opt_tagSize = 0.04;//tag side size in meters
bool display_tag = true;
float opt_quad_decimate = 2;

//Define the required homogeneous transform matrix
vpHomogeneousMatrix cdMo = vpHomogeneousMatrix(vpTranslationVector(0, 0, 0.21), vpRotationMatrix({ 0,-1,0,-1,0,0,0,0,-1 }));//Apriltag pose in the desired camera frame
vpHomogeneousMatrix cMo;//Apriltag code pose in the current camera frame
vpHomogeneousMatrix cdMc;//Current camera pose in the desired camera frame
vpHomogeneousMatrix vispRsim(vpTranslationVector(0, 0, 0), vpRotationMatrix({ -1,0,0,0,-1,0,0,0,1 }));//Homogeneous transform between ViSP and CoppeliaSim

//Visual servoing task
vpServo task;

//Choice of features for our visual servoing task
vpFeatureTranslation t(vpFeatureTranslation::cdMc);
vpFeatureThetaU tu(vpFeatureThetaU::cdRc);

//Force controller output signal (velocities m/s)
std::string fcl_sig;

//Error threshold
double convergence_threshold_t = 0.0005, convergence_threshold_tu = 0.1;

double lambda;
vpAdaptiveGain lambda_tu_ada(0.7, 0.3, 30);//adaptive gain using exponential law

//Choose desired control law (1: PBVS, 2: Hybrid parallel, 3: Hybrid extern, 4: Hybrid extern \w target locking)
int control_law_opt;

//function: vision_sensor_callback
//	@returns nothing: triggered at each simulation step. Processes image from CoppeliaSim's vision sensor and sends back camera kinematic screw to CoppeliaSim
void vision_sensor_callback(std::vector<msgpack::object>* msg)
{
	if (ready_for_servoing && !has_converged) {
		//Convert image to cv::Mat and then to vpImage
		std::string image_str = b0RemoteApi::readByteArray(msg, 2);
		cv::Mat image(height, width, CV_8UC1, const_cast<char*>(image_str.c_str()));
		cv::flip(image, image, 0);
		vpImageConvert::convert(image, I);

		vpDisplay::display(I);

		static bool start = false;
		if (!has_converged) {
			//Camera kinematic screw
			double C[6] = { 0,0,0,0,0,0 };

			//Resulting displacement from force controller
			cv::Mat dX(3, 1, CV_32FC1, const_cast<char*>(fcl_sig.c_str()));
			//If the operator hasn't started to control the robot yet, visual servoing won't begin
			if (dX.at<float>(0) != 0 || dX.at<float>(1) != 0 || dX.at<float>(2) != 0) {
				start = true;
			}
			std::vector<vpHomogeneousMatrix> cMo_vec;
			//Detect Apriltag and estimate its pose in camera frame 
			detector.detect(I, opt_tagSize, cam, cMo_vec);
			if (!cMo_vec.empty()) {
				cMo = cMo_vec[0];
				std::vector<float> cMo_vec(12);
				b0RemoteApi::readFloatArray(cl->simxGetObjectMatrix(43, 44, cl->simxServiceCall()), cMo_vec, 1);

				//Get the exact Apriltag pose in the current camera frame
				/*
				vpHomogeneousMatrix cMo_sim;
				int n_cols = 4;
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 4; j++) {
						cMo_sim[i][j] = cMo_vec[i * n_cols + j];
					}
				}
				cMo = cMo_sim;*/

				vpHomogeneousMatrix ds(vpTranslationVector(dX.at<float>(0), -dX.at<float>(1), -dX.at<float>(2)), vpRotationMatrix({ 1,0,0,0,1,0,0,0,1 }));
				cMo = vispRsim * cMo;

				vpDisplay::displayFrame(I, cMo, cam, opt_tagSize / 2, vpColor::none, 2);//Display Apriltag pose in current camera frame
				vpDisplay::displayFrame(I, cdMo, cam, opt_tagSize / 3, vpColor::none, 2);//Display Apriltag pose desired camera frame

				cdMc = cdMo * cMo.inverse();

				vpTranslationVector cd_t_c = cdMc.getTranslationVector();
				vpThetaUVector ThetaU = cdMc.getThetaUVector();

				lambda = lambda_tu_ada.value(cd_t_c.frobeniusNorm());

				double error_tr = sqrt(cd_t_c.sumSquare());
				double error_tu = vpMath::deg(sqrt(ThetaU.sumSquare()));

				if (error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu) {
					has_converged = true;
					std::cout << "Servo task has converged" << std::endl << std::endl;
					vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
					task.print();
					std::cout << std::endl;
				}

				//start "update task features" section
				switch (control_law_opt) {
					// PBVS
				case 1:
					t.buildFrom(cdMc);
					tu.buildFrom(cdMc);
					break;
					// Hybrid parallel
				case 2:
					tu.buildFrom(cdMc);
					break;
					// External hybrid
				case 3:
					t.buildFrom(ds);
					tu.buildFrom(cdMc);
					break;
					// External hybrid with target locking
				case 4:
					//Angle calculation for target locking
					double w_x = atan2(cMo[1][3], cMo[2][3]);
					double w_y = -atan2(cMo[0][3], cMo[2][3]);

					//Translation error, between desired camera frame and current camera frame, in Oxy plane
					double error_tr_x_y = sqrt(cd_t_c[0] * cd_t_c[0] + cd_t_c[1] * cd_t_c[1]);

					vpHomogeneousMatrix cdMc_lock(vpTranslationVector(0, 0, 0.05), // 0.05m along camera z axis
						vpThetaUVector(w_x, w_y, ThetaU[2]));
					t.buildFrom(ds);
					//If the translation error is less than the area defined by the apriltag, use visual servoing / else use target locking
					if (error_tr_x_y < sqrt(0.5) * opt_tagSize) {
						tu.buildFrom(cdMc);
					}
					else {
						tu.buildFrom(cdMc_lock);
					}
					break;
				}
				//end "update task features" section

				vpColVector v_c(6);
				v_c = task.computeControlLaw();//Compute camera velocity screw

				//If the operator started interacting with the robot, we can send rotational velocities
				if (start) {
					for (int i = 0; i < 6; i++) {
						C[i] = v_c[i];
						if (i > 2) {
							C[i] = lambda * C[i];
						}
					}
				}
			}

			// Hybrid parallel
			if (control_law_opt == 2) {
				for (int i = 0; i < 3; i++) {
					C[i] = dX.at<float>(i);
				}
			}

			std::stringstream packedArgs;
			msgpack::pack(packedArgs, C);//pack camera velocity screw, so we can send it to CoppeliaSim

			//Send velocity to server (coppeliasim)
			cl->simxCallScriptFunction("get_object_pose@Franka", "sim.scripttype_childscript", packedArgs.str().c_str(), packedArgs.str().size(), cl->simxServiceCall());

			vpDisplay::flush(I);
		}
	}

	do_next_step = true;
}

//function: simulation_initialization_callback
//	@param msg: msgpack vector
//	@returns nothing: change ready_for_servoing state to true when CoppeliaSim is ready.
void simulation_initialization_callback(std::vector<msgpack::object>* msg)
{
	if (b0RemoteApi::readString(msg, 1) == "ready")
		ready_for_servoing = true;
}

//function: fcl_callback
//	@param msg: msgpack vector
//	@returns nothing: update fcl_sig
void fcl_callback(std::vector<msgpack::object>* msg)
{
	fcl_sig = b0RemoteApi::readByteArray(msg, 1);
}

//function: step_simulation
//	@returns nothing: used to synchronize every calculation at each step
void step_simulation()
{
	while (!do_next_step) {
		cl->simxSpinOnce();
	}
	do_next_step = false;
	cl->simxSynchronousTrigger();
}

int main(int argc, char* argv[])
{
	//Define the client for communicating with CoppeliaSim
	b0RemoteApi client("b0RemoteApi_c++Client", "b0RemoteApi");
	cl = &client;

	//Set subscriber for getting the signal "franka". This signal is sent from CoppeliaSim when everything is initialized on the server side.
	client.simxGetStringSignal("franka", client.simxDefaultSubscriber(simulation_initialization_callback));

	//Set subscriber for the signal "fcl". This signal is used to get the input from the force controller, a proportional controller getting its input from the xbox controller.
	client.simxGetStringSignal("fcl", client.simxDefaultSubscriber(fcl_callback));

	//Retrieve vision sensor handle
	vision_sensor_handle = b0RemoteApi::readInt(client.simxGetObjectHandle("Vision_sensor0", client.simxServiceCall()), 1);

	//Set camera intrinsic parameters
	double u0 = width / 2;
	double v0 = height / 2;
	double px = 377;
	double py = 377;

	//Set camera intrinsics (perspective model without distorsion)
	cam = vpCameraParameters(px, py, u0, v0);

	//Get user input for desired control law
	while (true) {
		std::cout << "Choose desired control law (1: PBVS, 2 : Hybrid parallel, 3 : External hybrid, 4 : External hybrid \w target locking): ";
		std::cin >> control_law_opt;
		if (control_law_opt != 1 && control_law_opt != 2 && control_law_opt != 3 && control_law_opt != 4) {
			std::cout << "Wrong input" << std::endl;
		}
		else {
			break;
		}
	}
	//Task features
	task.addFeature(t);
	task.addFeature(tu);

	//Set servo type (eye-in-hand, velocities in operational space)
	task.setServo(vpServo::EYEINHAND_CAMERA);

	//Compute interaction matrix at each step
	task.setInteractionMatrixType(vpServo::CURRENT);

	//Detector instantiation
	detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
	detector.setDisplayTag(display_tag);
	detector.setAprilTagQuadDecimate(opt_quad_decimate);

	//Video display window
	I = vpImage<unsigned char>(height, width);
	vpDisplayOpenCV d(I);

	bool greyscale = true;

	//Subscriber for getting the vision sensor video stream in greyscale from the server (CoppeliaSim)
	client.simxGetVisionSensorImage(vision_sensor_handle, true, client.simxDefaultSubscriber(vision_sensor_callback, greyscale));

	client.simxSynchronous(true);

	client.simxStartSimulation(client.simxDefaultPublisher());

	long st = client.simxGetTimeInMs();

	while (!has_converged) {
		step_simulation();
	}

	client.simxStopSimulation(client.simxDefaultPublisher());

	system("pause");

	vpDisplay::close(I);

	return 0;
}
