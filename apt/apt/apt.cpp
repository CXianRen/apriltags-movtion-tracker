
/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#define no_init_all deprecated

#define HAVE_STRUCT_TIMESPEC
#include <iostream>
#include "opencv2/opencv.hpp"


extern "C" {
#include "apriltag.h"
#include "tag25h9.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}

using namespace std;
//using namespace cv;


int main(int argc, char *argv[])
{
	getopt_t *getopt = getopt_create();
	getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
	getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
	getopt_add_string(getopt, 'f', "family", "tag25h9", "Tag family to use");
	getopt_add_int(getopt, 't', "threads", "8", "Use this many CPU threads");
	getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
	getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
	getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

	// Initialize camera
	cv::VideoCapture cap(0);
	if (!cap.isOpened()) {
		cerr << "Couldn't open video capture device" << endl;
		return -1;
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);

	// Initialize tag detector with options
	apriltag_family_t *tf = NULL;
	const char *famname = getopt_get_string(getopt, "family");
	tf = tag25h9_create();
	

	apriltag_detector_t *td = apriltag_detector_create();
	apriltag_detector_add_family(td, tf);
	td->quad_decimate = getopt_get_double(getopt, "decimate");
	td->quad_sigma = getopt_get_double(getopt, "blur");
	td->nthreads = getopt_get_int(getopt, "threads");
	td->debug = getopt_get_bool(getopt, "debug");
	td->refine_edges = getopt_get_bool(getopt, "refine-edges");

	apriltag_detection_info_t info;
	info.tagsize = 0.01677;
	info.fx = 1974.973;
	info.fy = 1954.993;
	info.cx = 702.127;
	info.cy = 813.986;

	cv::Mat frame, gray;
	while (true) {
		cap >> frame;
		cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		// Make an image_u8_t header for the Mat data
		image_u8_t im = { gray.cols,
			gray.rows,
			gray.cols,
			gray.data
		};

		zarray_t *detections = apriltag_detector_detect(td, &im);
		cout << zarray_size(detections) << " tags detected" << endl;
		//
		double x[10] = { 0 }, y[10] = { 0 }, z[10] = {0};


		// Draw detection outlines
		for (int i = 0; i < zarray_size(detections); i++) {
			apriltag_detection_t *det;
			zarray_get(detections, i, &det);
			line(frame, cv::Point(det->p[0][0], det->p[0][1]),
				cv::Point(det->p[1][0], det->p[1][1]),
				cv::Scalar(0, 0xff, 0), 2);
			line(frame, cv::Point(det->p[0][0], det->p[0][1]),
				cv::Point(det->p[3][0], det->p[3][1]),
				cv::Scalar(0, 0, 0xff), 2);
			line(frame, cv::Point(det->p[1][0], det->p[1][1]),
				cv::Point(det->p[2][0], det->p[2][1]),
				cv::Scalar(0xff, 0, 0), 2);
			line(frame, cv::Point(det->p[2][0], det->p[2][1]),
				cv::Point(det->p[3][0], det->p[3][1]),
				cv::Scalar(0xff, 0, 0), 2);

			stringstream ss;
			ss << det->id;
			cv::String text = ss.str();
			int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
			double fontscale = 1.0;
			int baseline;
			cv::Size textsize = getTextSize(text, fontface, fontscale, 2,
				&baseline);
			putText(frame, text, cv::Point(det->c[0] - textsize.width / 2,
				det->c[1] + textsize.height / 2),
				fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
			//calculate pose
			apriltag_pose_t pose;
			info.det = det;
			double err = estimate_tag_pose(&info, &pose);
			//cout << "T:[" << pose.t->nrows<<","<< pose.t->ncols << "]" << endl;
			//for (int m = 0; m < pose.t->nrows; m++) {
			//	cout << "[";
			//	for (int j = 0; j < pose.t->ncols; j++)
			//		cout << pose.t->data[m*pose.t->ncols + j]<<",";
			//	cout << "]"<<endl;
			//}
			x[i] = pose.t->data[0];
			y[i] = pose.t->data[1];
			z[i] = pose.t->data[2];
		}
		if (zarray_size(detections) >= 2) {
			float dis = sqrtf(pow(x[0] - x[1],2) + pow(y[0] - y[1],2) );
			cout << "dist:" << dis << endl;
		}

		apriltag_detections_destroy(detections);

		imshow("Tag Detections", frame);
		if (cv::waitKey(5) >= 0)
			break;
	}

	apriltag_detector_destroy(td);
	tag25h9_destroy(tf);
	getopt_destroy(getopt);

	return 0;
}
