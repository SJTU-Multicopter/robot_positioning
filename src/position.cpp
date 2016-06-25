#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <math.h>

#define CV_RED cvScalar(0,0,255,0)
#define CV_GREEN cvScalar(0,255,0,0)
#define CV_WHITE cvScalar(255,255,255,0)
#define CV_BLACK cvScalar(0,0,0,0)

#define THRESHOLD 60

#define image_width 1280
#define image_height 720

bool camera_left_side = true;
bool camera_enermy_side = true;

using namespace std;
using namespace cv;

float point_distance(int x1, int y1, int x2, int y2);
float point_distance_f(float x1, float y1, float x2, float y2);

CvMat* find_useful_lines_kb(CvSeq* lines_s, int width, int height, 
	int length = 50, float delt_k = 0.2, float delt_d_n = 20, float delt_d_p = 20); // delt_b=(0,1)
CvPoint2D32f lines_cross_point(float k1, float b1, float k2, float b2);
void draw_result_lines(CvMat *lines_mat, IplImage* image, CvScalar color = CV_RED);
bool f_euqal(float a, float b);

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
	IplImage* image_raw = cvCreateImage(cvSize(image_width,image_height),8,3);//4:3画面
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
    cout<<"fill_color->height="<<fill_color->height<<endl;

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


    /*****Find blue lines ******/
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
    cout<<"total lines "<<lines_blue->total<<endl;

    for (int i = 0; i < lines_blue->total; i++)  
    {  
        //point: line[0] and line[1]
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines_blue, i);  
        cvLine(color_blue, line[0], line[1], CV_RGB(30, 144, 255), 2, CV_AA, 0);  
    }


    /***Find the useful lines***/
    CvMat* blue_lines_mat = find_useful_lines_kb(lines_blue, image_width, image_height, 60, 0.2, 20, 80);
    draw_result_lines(blue_lines_mat, color_blue);
    
    /***Cut the useless area***/
    float n_b_max = -100000.f;  //k is negative, value b when b is max
    float n_k_max = 0.f;  //k is negative, value k when b is max
    float n_b_min = 100000.f;
    float n_k_min = 0.f; 
    float p_b_max = -100000.f;
    float p_k_max = 0.f;
    float p_b_min = 100000.f;
    float p_k_min = 0.f; 

    for(int i = 1; i <= (int)CV_MAT_ELEM(*blue_lines_mat, float, 0, 0); i++)
    {
    	if(CV_MAT_ELEM(*blue_lines_mat, float, i, 0) < 0)
    	{
    		if(CV_MAT_ELEM(*blue_lines_mat, float, i, 1) > n_b_max) {
    			n_k_max = CV_MAT_ELEM(*blue_lines_mat, float, i, 0);
    			n_b_max = CV_MAT_ELEM(*blue_lines_mat, float, i, 1);
    		}
    		if(CV_MAT_ELEM(*blue_lines_mat, float, i, 1) < n_b_min) {
    			n_k_min = CV_MAT_ELEM(*blue_lines_mat, float, i, 0);
    			n_b_min = CV_MAT_ELEM(*blue_lines_mat, float, i, 1);
    		}
    	}
    	else
    	{
    		if(CV_MAT_ELEM(*blue_lines_mat, float, i, 1) > n_b_max) {
    			p_k_max = CV_MAT_ELEM(*blue_lines_mat, float, i, 0);
    			p_b_max = CV_MAT_ELEM(*blue_lines_mat, float, i, 1);
    		}
    		if(CV_MAT_ELEM(*blue_lines_mat, float, i, 1) < n_b_min) {
    			p_k_min = CV_MAT_ELEM(*blue_lines_mat, float, i, 0);
    			p_b_min = CV_MAT_ELEM(*blue_lines_mat, float, i, 1);
    		}
    	}
    }

    if(n_b_max > -100000.f && n_b_min < 100000.f && fabs(n_b_max - n_b_min) > 100.f)
    {
	    CvScalar s2;

	    for(int i = 0;i < fill_color->height;i++)
	    {
	        for(int j = 0;j < fill_color->width;j++)
	        {
	            int x = j;
	            int y = image_height - i;
	            if((n_k_max*x-y+n_b_max) < 0 || (n_k_min*x-y+n_b_min) > 0)
	            {
	            	s2 = cvGet2D(fill_color,i,j); // get the (i,j) pixel value, rows, cols
		            s2.val[0]=169;
		            s2.val[1]=169;
		            s2.val[2]=169;
		            cvSet2D(fill_color,i,j,s2);//set the (i,j) pixel value
	            }   
	        }
	    }
    }

    if(p_b_max > -100000.f && p_b_min < 100000.f && fabs(p_b_max - p_b_min) > 100.f)
    {
	    CvScalar s3;

	    for(int i = 0;i < fill_color->height;i++)
	    {
	        for(int j = 0;j < fill_color->width;j++)
	        {
	            int x = j;
	            int y = image_height - i;
	            if((p_k_max*x-y+p_b_max) > 0 || (p_k_min*x-y+p_b_min) > 0)
	            {
	            	s3 = cvGet2D(fill_color,i,j); // get the (i,j) pixel value, rows, cols
		            s3.val[0]=169;
		            s3.val[1]=169;
		            s3.val[2]=169;
		            cvSet2D(fill_color,i,j,s3);//set the (i,j) pixel value
	            }   
	        }
	    }
    }



    


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

    cvReleaseMat(&blue_lines_mat);

  	cout<<"finished"<<endl;
	
	return 0;
}

float point_distance(int x1, int y1, int x2, int y2)
{
	return sqrt((float)(x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

float point_distance_f(float x1, float y1, float x2, float y2)
{
	return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

bool f_euqal(float a, float b)
{
	if(fabs(a-b)<0.0001) return true;
	else return false;
}

CvPoint2D32f lines_cross_point(float k1, float b1, float k2, float b2)
{
	CvPoint2D32f p;
	p.x = (b2 - b1)/(k1 - k2);
	p.y = k1*p.x + b1;

	return p;
}

CvMat* find_useful_lines_kb(CvSeq* lines_s, int width, int height, int length, float delt_k , float delt_d_n, float delt_d_p)  //return cvmat[0] means total number
{
	int lines_num = lines_s->total;
    float lines[lines_num][2];
    
    int counter = 0;
    for (int i = 0; i < lines_s->total; i++)  
    {  
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines_s, i);  
        
        if(point_distance(line[0].x, line[0].y, line[1].x, line[1].y) > length) // by length
        {
        	// To normal coordinate
        	float x1 = line[0].x; 
        	float y1 = height-line[0].y; 
        	float x2 = line[1].x;
        	float y2 = height-line[1].y; 

        	lines[counter][0] = (y2-y1)/(x2-x1);
        	lines[counter][1] = y1 - lines[counter][0]*x1;
        	counter ++;
        }
        else lines_num --;
    }
    
    for(int i = 0; i < lines_num - 1; i++)
    {
    	for(int j = i+1; j < lines_num; j++)
    	{
  
    		if(fabs(lines[i][0]-lines[j][0]) < delt_k)  //fabs(lines[i][1]-lines[j][1])/fabs(lines[j][1]) < delt_b)
    		{
    			float k1 = (lines[i][0] + lines[j][0])/2.f;
    			float k2 = -1.f/k1;
    			float b = height/2.f - k2*width/2.f;
                CvPoint2D32f cross_point_1 = lines_cross_point(k1, lines[i][1], k2, b);
                CvPoint2D32f cross_point_2 = lines_cross_point(k1, lines[j][1], k2, b);
                float distance = point_distance_f(cross_point_1.x, cross_point_1.y, cross_point_2.x, cross_point_2.y);

    			if(k1 < 0 && distance < delt_d_n)
    			{
    				lines[i][0] = k1;
					lines[i][1] = (lines[i][1] + lines[j][1])/2;
					//reset sequence
					for(int h = j+1; h < lines_num; h++)
					{
						lines[h-1][0] = lines[h][0];
						lines[h-1][1] = lines[h][1];
					}
					lines_num --;
					j--;
    			}
    			else if(k1 > 0 && distance < delt_d_p)
    			{
    				lines[i][0] = k1;
					lines[i][1] = (lines[i][1] + lines[j][1])/2;
					//reset sequence
					for(int h = j+1; h < lines_num; h++)
					{
						lines[h-1][0] = lines[h][0];
						lines[h-1][1] = lines[h][1];
					}
					lines_num --;
					j--;
    			}
    			else ;	
    		}
    	}
    }


    cout<<"lines_num = "<<lines_num<<endl;

    CvMat* lines_mat = cvCreateMat(lines_num+1,2,CV_32FC1);
    
    CV_MAT_ELEM(*lines_mat, float, 0, 0 ) = (float)lines_num;
    CV_MAT_ELEM(*lines_mat, float, 0, 1 ) = 0.f;

    for(int i=0;i<lines_num;i++)
    {
    	CV_MAT_ELEM(*lines_mat, float, i+1, 0 ) = lines[i][0];
    	CV_MAT_ELEM(*lines_mat, float, i+1, 1 ) = lines[i][1];
    }

    return lines_mat;
}


void draw_result_lines(CvMat *lines_mat, IplImage* image, CvScalar color)
{
	for(int i = 1; i <= CV_MAT_ELEM(*lines_mat, float, 0, 0 ); i++)
    {
    	CvPoint p1;
    	CvPoint p2;
    	bool p1_set = false;
    	//cross point with x = 0 and x = width
    	int y1 = CV_MAT_ELEM(*lines_mat, float, i, 1 );
    	if(y1 > 0 && y1 < image_height) { 
    		p1.x = 0; 
    		p1.y = y1;
    		p1_set = true;
    	}
    	int y2 = CV_MAT_ELEM(*lines_mat, float, i, 0 )*image_width+CV_MAT_ELEM(*lines_mat, float, i, 1 );
    	if(y2 > 0 && y2 < image_height) { 
    		if(p1_set){
    			p2.x = image_width; 
    			p2.y = y2;
    			//continue;
    		}
    		else
    		{
    			p1.x = image_width; 
    			p1.y = y2;
    			p1_set = true;
    		}
    	}
    	//cross point with y = 0 and y = height
    	int x3 = (0-CV_MAT_ELEM(*lines_mat, float, i, 1 ))/CV_MAT_ELEM(*lines_mat, float, i, 0 );
    	if(x3 > 0 && x3 < image_width) { 
    		if(p1_set){
    			p2.x = x3; 
    			p2.y = 0;
    			//continue;
    		}
    		else
    		{
    			p1.x = x3; 
    			p1.y = 0;
    			p1_set = true;
    		}
    	}
    	int x4 = (image_height - CV_MAT_ELEM(*lines_mat, float, i, 1 ))/CV_MAT_ELEM(*lines_mat, float, i, 0 );
    	if(x4 > 0 && x4 < image_width) { 
    			p2.x = x4; 
    			p2.y = image_height;
    	}

    	//translate to image coordinate
        p1.y = image_height - p1.y;
        p2.y = image_height - p2.y;
    	cvLine(image, p1, p2 , color, 1, CV_AA, 0);
    }
}