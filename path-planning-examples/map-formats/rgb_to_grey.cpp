//Include Libraries
#include<opencv2/opencv.hpp>
#include<iostream>

// Namespace nullifies the use of cv::function(); 
using namespace std;
using namespace cv;

int main()
{
	// Read an image 
	Mat img_grayscale = imread("simple-house-design-lv1.png", IMREAD_GRAYSCALE);
	Mat dst;
	int threshold_value = 0;
	int threshold_type = 3;
	int const max_value = 255;
	int const max_type = 4;
	int const max_binary_value = 255;
	// Display the image.
	imshow("grayscale image", img_grayscale); 

	threshold( img_grayscale, dst, threshold_value, max_binary_value, threshold_type );
	uint8_t *myData = img_grayscale.data;
	int width = img_grayscale.cols;
	int height = img_grayscale.rows;
	int _stride = img_grayscale.step;//in case cols != strides
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			uint8_t val = myData[ i * _stride + j];
			cout << val << " ";
		}
		cout << endl;
	}

	// Wait for a keystroke.   
	waitKey(0);  
	// Destroys all the windows created                         
	destroyAllWindows();

	// Write the image in the same directory
	imwrite("gray_map.pgm", img_grayscale);

	return 0;
}


/*
Source:					https://learnopencv.com/read-display-and-write-an-image-using-opencv/
						https://docs.opencv.org/4.x/db/deb/tutorial_display_image.html 
Basic Op:				https://docs.opencv.org/4.x/de/d06/tutorial_js_basic_ops.html 
Show image in matrix: 	https://stackoverflow.com/questions/1844736/accessing-a-matrix-element-in-the-mat-object-not-the-cvmat-object-in-opencv
*/