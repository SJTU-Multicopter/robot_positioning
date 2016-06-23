#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include <iostream>

#define CV_RED cvScalar(255,0,0,0)
#define CV_GREEN cvScalar(0,255,0,0)
#define CV_WHITE cvScalar(255,255,255,0)
#define CV_BLACK cvScalar(0,0,0,0)

#define THRESHOLD 60

using namespace std;
using namespace cv;

int color_threshold[5][6]={
	60,130,50,255,50,255,
	20,35,140,255,180,255,
	0,20,150,255,150,255,
	0,0,0,0,0,0,
	0,0,0,0,0,0
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "position");

	ros::NodeHandle nh;


	char image_name[100] = "/home/chg/catkin_ws/src/robot_positioning/images/IMG_20160616_152830.jpg";
	
	IplImage* image_origin = cvLoadImage(image_name);
	IplImage* image_raw = cvCreateImage(cvSize(1280,720),8,3);//4:3画面
    cvResize(image_origin,image_raw,CV_INTER_NN);
    cvSmooth(image_raw,image_raw,CV_BLUR,3,3);
    
	//hsv空间下分离
    IplImage* hsv = cvCreateImage(cvGetSize(image_raw), 8, 3);
    cvCvtColor(image_raw, hsv, CV_BGR2HSV);
     

     //创建灰色图像
    IplImage* fill_color = cvCreateImage(cvGetSize(image_raw), 8, 3);
    CvScalar s;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            //s = cvGet2D(fill_color,i,j); // get the (i,j) pixel value
            s.val[0]=169;
            s.val[1]=169;
            s.val[2]=169;
            cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
        }
    }

    //Blue
    CvScalar s_blue;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s_blue = cvGet2D(hsv,i,j); // get the (i,j) pixel value
            if(s_blue.val[0] >= color_threshold[0][0] && s_blue.val[0] <= color_threshold[0][1] &&
                    s_blue.val[1] >= color_threshold[0][2] && s_blue.val[1] <= color_threshold[0][3] &&
                    s_blue.val[2] >= color_threshold[0][4] && s_blue.val[2] <= color_threshold[0][5])
            {
                s.val[0] = 255;
                s.val[1] = 144;
                s.val[2] = 30;
                cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
            }

        }
    }

	//Yellow
    CvScalar s_yellow;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s_yellow = cvGet2D(hsv,i,j); // get the (i,j) pixel value
            if(s_yellow.val[0] >= color_threshold[1][0] && s_yellow.val[0] <= color_threshold[1][1] &&
                    s_yellow.val[1] >= color_threshold[1][2] && s_yellow.val[1] <= color_threshold[1][3] &&
                    s_yellow.val[2] >= color_threshold[1][4] && s_yellow.val[2] <= color_threshold[1][5])
            {
                s.val[0] = 86;
                s.val[1] = 255;
                s.val[2] = 255;
                cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
            }

        }
    }


    //Red
    CvScalar s_red;

    for(int i = 0;i < fill_color->height;i++)
    {
        for(int j = 0;j < fill_color->width;j++)
        {
            s_red = cvGet2D(hsv,i,j); // get the (i,j) pixel value
            if(s_red.val[0] >= color_threshold[2][0] && s_red.val[0] <= color_threshold[2][1] &&
                    s_red.val[1] >= color_threshold[2][2] && s_red.val[1] <= color_threshold[2][3] &&
                    s_red.val[2] >= color_threshold[2][4] && s_red.val[2] <= color_threshold[2][5])
            {
                s.val[0] = 0;
                s.val[1] = 0;
                s.val[2] = 255;
                cvSet2D(fill_color,i,j,s);//set the (i,j) pixel value
            }

        }
    }


    /*****Find blue lines and circle******/
    CvScalar s_blue_black;
    IplImage* blue = cvCreateImage(cvGetSize(fill_color), 8, 1);

    for(int i = 0;i < blue->height;i++)
    {
        for(int j = 0;j < blue->width;j++)
        {
            s_blue_black = cvGet2D(fill_color,i,j); // get the (i,j) pixel value
            if(s_blue_black.val[0] == 255 && s_blue_black.val[1] == 144 && s_blue_black.val[2] == 30)
            {
                cvSet2D(blue,i,j,CV_WHITE);//set the (i,j) pixel value
            }
            else 
            {
                cvSet2D(blue,i,j,CV_BLACK);//set the (i,j) pixel value
            }
        }
    }

    cvErode( blue,blue, NULL, 1);   
    cvDilate( blue,blue, NULL, 3);
    
    //find lines
    IplImage* blue_canny = cvCreateImage(cvGetSize(blue), 8, 1);
    cvCanny(blue, blue_canny, 100, 200, 3);

    CvMemStorage* storage_blue_canny = cvCreateMemStorage(0);  
    CvSeq* lines_blue = 0; 

    IplImage* color_blue = cvCreateImage(cvGetSize(blue_canny), 8, 3); 
    cvCvtColor( blue_canny, color_blue, CV_GRAY2BGR ); 

    lines_blue = cvHoughLines2(blue_canny, storage_blue_canny, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 60, 50, 40);  
    for (int i = 0; i < lines_blue->total; i++)  
    {  
        //point: line[0] and line[1]
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines_blue, i);  
        cvLine(color_blue, line[0], line[1], CV_RGB(30, 144, 255), 3, CV_AA, 0);  
    }
    

    // find ellipse
    /*CvMemStorage* storage_blue_contour= cvCreateMemStorage(0);
    CvSeq* contours_blue;
    CvBox2D box2d_blue;

    cvFindContours( blue, storage_blue_contour, &contours_blue, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));  
    
    
    box2d_blue=cvFitEllipse2(contours_blue); 
    box2d_blue.angle=90-box2d_blue.angle;//这个地方是OPENCV的一个BUG 角度要修正一下

    cvEllipseBox(color_blue,box2d_blue, CV_RGB(30,144,255),3, 8 , 0 );//画椭圆框*/ 



	cvNamedWindow("image_raw");
	cvShowImage("image_raw",image_raw);
	cvNamedWindow("fill_color");
	cvShowImage("fill_color",fill_color);
	cvNamedWindow("blue");
	cvShowImage("blue",blue);
	cvNamedWindow("blue_canny");
	cvShowImage("blue_canny",blue_canny);
	cvNamedWindow("color_blue");
	cvShowImage("color_blue",color_blue);
    
    cvWaitKey(0);

    cvDestroyWindow("image_raw");
    cvReleaseImage(&image_raw);
    cvDestroyWindow("fill_color");
    cvReleaseImage(&fill_color);
    cvDestroyWindow("blue");
    cvReleaseImage(&blue);
    cvDestroyWindow("blue_canny");
    cvReleaseImage(&blue_canny);
    cvDestroyWindow("color_blue");
    cvReleaseImage(&color_blue);

  	cout<<"finished"<<endl;
	
	return 0;
}



	
