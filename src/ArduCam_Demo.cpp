// ArduCam_test.cpp : Defines the entry point for the console application.
//
#ifdef __linux__
#include <ArduCamLib.h>
#include <unistd.h>
#include <termios.h>
#endif

#ifdef _WIN32
#include "stdafx.h"
#include <Windows.h>
#include "ArduCamlib.h"
#include <io.h>
#include <direct.h> 
#include <conio.h>
#endif
#include <opencv2/opencv.hpp>
#include <thread>
#include <time.h>
#include <iostream>
#include <istream>
#include <string>
#include "Utils.h"
#include "Arducam.hpp"

#include "arducam_rgbir_debayer.h"

using namespace std;
using namespace cv;

void delay_ms(int mills) {
#ifdef __linux__
	usleep(mills * 1000);
#endif
#ifdef _WIN32
	Sleep(mills);
#endif
}

static void display_fps(int index) {
	static thread_local int frame_count = 0;
	static thread_local time_t start = time(NULL);

	frame_count++;

	time_t end = time(NULL);
	if (end - start >= 1) {
		start = end;
		std::cout << "camera: "<< index <<  ", fps: " << frame_count << std::endl;
		frame_count = 0;
	}
}

void display(ArducamCamera *camera, int index) {
	cv::Mat image;
	uint32_t row = camera->cameraCfg.u32Height;
	uint32_t col = camera->cameraCfg.u32Width;
	uint32_t bitWidth = camera->cameraCfg.u8PixelBits;
	uint16_t* pu16ImageData = (uint16_t*)malloc(row * col * sizeof(uint16_t));
	while (true) {
		ArduCamOutData* frameData;
		if (!camera->read(frameData)) {
			std::cout << "read frame failed." << std::endl;
			continue;
		}

		if ((int)camera->cameraCfg.emImageFmtMode == 9) {
			Convert8To16Buffer(pu16ImageData, frameData->pu8ImageData, frameData->stImagePara.u8PixelBits, frameData->stImagePara.u32Size);
			std::tuple<std::vector<uint16_t>, std::vector<uint16_t>> results = processRgbIr16BitData(pu16ImageData, row, col, camera->color_mode);
			cv::Mat rgb_img = RGBToMat(std::get<0>(results).data(), col, row);
			cv::Mat ir_full_img = IRToMat(std::get<1>(results).data(), col, row);
			cv::hconcat(rgb_img, ir_full_img, image);
		} else {
			image = ConvertImage(frameData, camera->color_mode);
		}
		camera->returnFrameBuffer();
		if (!image.data) {
			std::cout << "No image data" << std::endl;
			continue;
		}
		display_fps(index);

		double scale = 640.0 / image.cols;
		cv::resize(image, image,
			cv::Size(), scale, scale, cv::INTER_LINEAR);

		std::string name = "ArduCam";
		name += std::to_string(index);
		cv::imshow(name, image);

		int key = -1;
		key = cv::waitKey(1);
		if (key == 'q')
			break;
	}

	free(pu16ImageData);

	camera->stop();

	camera->closeCamera();
}

int main(int argc,char **argv)
{
	const char * config_file_name;
	int deviceID;
	if(argc == 2){
		config_file_name = argv[1];
		deviceID = 0;
	} else if (argc > 2) {
		config_file_name = argv[1];
		deviceID = atoi(argv[2]);
	} else {
		showHelp();
		return 0;
	}


	ArducamCamera *camera = new ArducamCamera();
	printf("Select Device ID: %d\n", deviceID);
	if (!camera->openCamera(config_file_name, deviceID)) {
		std::cout << "Failed to open camera." << std::endl;
		return 0;
	}

	camera->dumpDeviceInfo();
	camera->start();
	std::thread camera_display(display, camera, deviceID);

	camera->setCtrl("setFramerate", 15);
	camera->setCtrl("setExposureTime", 200000);

	camera_display.join();
	return 0;
}


