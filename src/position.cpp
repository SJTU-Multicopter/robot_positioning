#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <math.h>

#define CV_RED cvScalar(0,0,255,0)
#define CV_GREEN cvScalar(0,255,0,0)
#define CV_WHITE cvScalar(255,255,255,0)
#define CV_BLACK cvScalar(0,0,0,0)
#define CV_YELLOW cvScalar(0,255,255,0)
#define CV_BLUE cvScalar(255,0,0,0)

#define THRESHOLD 60

#define image_width 1280
#define image_height 720

bool camera_left_side = false;
bool camera_enemy_side = true;

float robot_image_p[2] = {0.f, 0.f};
float robot_real_p[2] = {0.f, 0.f};

using namespace std;
using namespace cv;

float point_distance(int x1, int y1, int x2, int y2);

float point_distance_f(float x1, float y1, float x2, float y2);

CvMat* find_useful_lines_kb(CvSeq* lines_s, int width, int height, 
	int length = 50, float delt_k = 0.2, float delt_d_n = 20, float delt_d_p = 20); // delt_b=(0,1)

CvPoint2D32f lines_cross_point(float k1, float b1, float k2, float b2);

void draw_result_lines(CvMat *lines_mat, IplImage* image, CvScalar color = CV_RED, int thickness = 2);

bool f_euqal(float a, float b);

CvPoint2D32f point_slant_coordinate_tranlate(float kx, float ky, float x0, float y0);

int color_threshold[5][6]={
	60,130,50,255,50,255,
	20,35,140,255,180,255,
	0,20,150,255,150,255,
	0,0,0,0,0,0,
	0,0,0,0,0,0
};



float cross_points_real_position_right_enemy[5][5][2] = {
	5.f, 2.f, 5.f, 1.f, 5.f, 0.f, 5.f, -1.f, 5.f, -2.f,
	4.f, 2.f, 4.f, 1.f, 4.f, 0.f, 4.f, -1.f, 4.f, -2.f,
	3.f, 2.f, 3.f, 1.f, 3.f, 0.f, 3.f, -1.f, 3.f, -2.f,
	2.f, 2.f, 2.f, 1.f, 2.f, 0.f, 2.f, -1.f, 2.f, -2.f,
	1.f, 2.f, 1.f, 1.f, 1.f, 0.f, 1.f, -1.f, 1.f, -2.f
};

float cross_points_real_position_left_enemy[5][5][2];


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

    /***Draw lines**/
    /*for (int i = 0; i < lines_blue->total; i++)  
    {  
        //point: line[0] and line[1]
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines_blue, i);  
        cvLine(color_blue, line[0], line[1], CV_BLUE, 1, CV_AA, 0);  
    }*/


    /***Find the useful lines***/
    CvMat* blue_lines_mat = find_useful_lines_kb(lines_blue, image_width, image_height, 60, 0.2, 20, 80);
    draw_result_lines(blue_lines_mat, color_blue, CV_BLUE);
    
    /***Cut the useless area***/
    float n_b_max = -100000.f;  //k is negative, value b when b is max
    float n_k_max = 0.f;  //k is negative, value k when b is max
    float n_b_min = 100000.f;
    float n_k_min = 0.f; 
    float p_b_max = -100000.f;
    float p_k_max = 0.f;
    float p_b_min = 100000.f;
    float p_k_min = 0.f; 

    for(int i = 1; i < CV_MAT_ELEM(*blue_lines_mat, float, 0, 0) + 1; i++)
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
		            s2.val[0]=100;
		            s2.val[1]=100;
		            s2.val[2]=100;
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
		            s3.val[0]=100;
		            s3.val[1]=100;
		            s3.val[2]=100;
		            cvSet2D(fill_color,i,j,s3);//set the (i,j) pixel value
	            }   
	        }
	    }
    }

    /***Find robot***/
    CvScalar s_red_black;
    IplImage* red = cvCreateImage(cvGetSize(fill_color), 8, 1);

    for(int i = 0;i < red->height;i++)
    {
        for(int j = 0;j < red->width;j++)
        {
            s_red_black = cvGet2D(fill_color,i,j); // get the (i,j) pixel value
            if(s_red_black.val[0] == 0 && s_red_black.val[1] == 0 && s_red_black.val[2] == 255)
            {
                cvSet2D(red,i,j,CV_WHITE);//set the (i,j) pixel value
            }
            else 
            {
                cvSet2D(red,i,j,CV_BLACK);//set the (i,j) pixel value
            }
        }
    }
    

    cvErode( red, red, NULL, 1);   
    cvDilate( red, red, NULL, 1);

    CvPoint2D32f robot_center;  
    float radius;

    CvMemStorage* red_storage = cvCreateMemStorage(0);
    CvSeq* red_contours = 0;
    int contour_num = cvFindContours(red, red_storage, &red_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cout<<"contour_num="<<contour_num<<endl;
    if(contour_num > 0)
    {
    	cvDrawContours(fill_color, red_contours, CV_GREEN, CV_GREEN, 50);

    	CvSeq *c = 0;
    	bool found = false;
    	for (c = red_contours;c !=NULL;c = c->h_next)
    	{
    		double area = fabs(cvContourArea(c,CV_WHOLE_SEQ));
    		if(area > 200)
    		{
    			cvMinEnclosingCircle(c,&robot_center,&radius); 
				cvCircle(fill_color,cvPointFrom32f(robot_center),4,CV_GREEN,4);
				robot_image_p[0] = robot_center.x;
				robot_image_p[1] = image_height - robot_center.y;
				found = true;
    		}
    		if(!found) 
    		{
    			robot_image_p[0] = 0.f;
				robot_image_p[1] = 0.f;
    		}
    	}
    }
    else
    {
    	robot_image_p[0] = 0.f;
		robot_image_p[1] = 0.f;
    }


    /*****Find yellow lines ******/
    CvScalar s_yellow_black;
    IplImage* yellow = cvCreateImage(cvGetSize(fill_color), 8, 1);

    for(int i = 0;i < yellow->height;i++)
    {
        for(int j = 0;j < yellow->width;j++)
        {
            s_yellow_black = cvGet2D(fill_color,i,j); // get the (i,j) pixel value
            if(s_yellow_black.val[0] == 86 && s_yellow_black.val[1] == 255 && s_yellow_black.val[2] == 255)
            {
                cvSet2D(yellow,i,j,CV_WHITE);//set the (i,j) pixel value
            }
            else 
            {
                cvSet2D(yellow,i,j,CV_BLACK);//set the (i,j) pixel value
            }
        }
    }

    cvErode( yellow,yellow, NULL, 1);   
    cvDilate( yellow,yellow, NULL, 1);
    
    //find lines
    IplImage* yellow_canny = cvCreateImage(cvGetSize(yellow), 8, 1);
    cvCanny(yellow, yellow_canny, 100, 200, 3);

    CvMemStorage* storage_yellow_canny = cvCreateMemStorage(0);  
    CvSeq* lines_yellow = 0; 

    IplImage* color_yellow = cvCreateImage(cvGetSize(yellow_canny), 8, 3); 
    cvCvtColor( yellow_canny, color_yellow, CV_GRAY2BGR ); 

    lines_yellow = cvHoughLines2(yellow_canny, storage_yellow_canny, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 60, 50, 40);  
    cout<<"total lines "<<lines_yellow->total<<endl;

    /**Draw lines**/
    /*for (int i = 0; i < lines_yellow->total; i++)  
    {  
        //point: line[0] and line[1]
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines_yellow, i);  
        cvLine(color_yellow, line[0], line[1], CV_YELLOW, 1, CV_AA, 0);  
    }*/


    /***Find the useful lines***/
    CvMat* yellow_lines_mat = find_useful_lines_kb(lines_yellow, image_width, image_height, 60, 0.2, 20, 20);
    draw_result_lines(yellow_lines_mat, color_yellow, CV_YELLOW);


    /***Classify negtive k lines and positive k lines***/
    int yellow_lines_total = (int)CV_MAT_ELEM(*yellow_lines_mat, float, 0, 0 );

    float negative_yellow_lines_array[yellow_lines_total][2];  //k,b  
    float positive_yellow_lines_array[yellow_lines_total][2];  //k,b
    int negative_k_total = 0;
    int positive_k_total = 0;

    float negative_k_average = 0.f;
    float positive_k_average = 0.f;

    for(int i = 1; i <= yellow_lines_total; i++)
    {
    	if(CV_MAT_ELEM(*yellow_lines_mat, float, i, 0 ) < 0.f)
    	{
    		negative_yellow_lines_array[negative_k_total][0] = CV_MAT_ELEM(*yellow_lines_mat, float, i, 0 );
    		negative_yellow_lines_array[negative_k_total][1] = CV_MAT_ELEM(*yellow_lines_mat, float, i, 1 );
    		negative_k_average += negative_yellow_lines_array[negative_k_total][0];
    		negative_k_total ++;
    		//cout<<"nk ("<<negative_yellow_lines_array[negative_k_total][0]<<","<<negative_yellow_lines_array[negative_k_total][1]<<")\n";
    	}
    } 
    negative_k_average = negative_k_average / negative_k_total;

    for(int i = 1; i <= yellow_lines_total; i++)
    {
    	if(CV_MAT_ELEM(*yellow_lines_mat, float, i, 0 ) >= 0.f)
    	{
    		positive_yellow_lines_array[positive_k_total][0] = CV_MAT_ELEM(*yellow_lines_mat, float, i, 0 );
    		positive_yellow_lines_array[positive_k_total][1] = CV_MAT_ELEM(*yellow_lines_mat, float, i, 1 );
    		positive_k_average += positive_yellow_lines_array[positive_k_total][0];
    		positive_k_total ++;
    		//cout<<"pk ("<<positive_yellow_lines_array[i-1][0]<<","<<positive_yellow_lines_array[i-1][1]<<")\n";
    	}
    }
    positive_k_average = positive_k_average / positive_k_total;

    cout<<"negative k lines: "<<negative_k_total<<endl;
    cout<<"positive k lines: "<<positive_k_total<<endl;

    /***Rank array by b, from large to small***/
    for(int i = 0; i < negative_k_total - 1; i++)
    {
    	for(int j = i + 1; j < negative_k_total; j++)
    	{
    		if(negative_yellow_lines_array[i][1] < negative_yellow_lines_array[j][1])
    		{
    			float temp1 = negative_yellow_lines_array[j][1];
    			float temp2 = negative_yellow_lines_array[j][0];
    			negative_yellow_lines_array[j][1] = negative_yellow_lines_array[i][1];
    			negative_yellow_lines_array[j][0] = negative_yellow_lines_array[i][0];
    			negative_yellow_lines_array[i][1] = temp1;
    			negative_yellow_lines_array[i][0] = temp2;
    		}
    	}
    }

    for(int i = 0; i < positive_k_total - 1; i++)
    {
    	for(int j = i + 1; j < positive_k_total; j++)
    	{
    		if(positive_yellow_lines_array[i][1] < positive_yellow_lines_array[j][1])
    		{
    			float temp1 = positive_yellow_lines_array[j][1];
    			float temp2 = positive_yellow_lines_array[j][0];
    			positive_yellow_lines_array[j][1] = positive_yellow_lines_array[i][1];
    			positive_yellow_lines_array[j][0] = positive_yellow_lines_array[i][0];
    			positive_yellow_lines_array[i][1] = temp1;
    			positive_yellow_lines_array[i][0] = temp2;
    		}
    	}
    }

    /***Calculate Cross Points***/
    int max_points_number = negative_k_total*positive_k_total;
    int cross_points_yellow[max_points_number][2]; //(x,y)

    for(int m = 0; m < negative_k_total; m++)
    {
    	for(int n = 0; n < positive_k_total; n++)
    	{
    	    float k1 = negative_yellow_lines_array[m][0];
    	    float b1 = negative_yellow_lines_array[m][1];
    	    float k2 = positive_yellow_lines_array[n][0];
    	    float b2 = positive_yellow_lines_array[n][1];
    	    //cout<<"("<<negative_yellow_lines_array[m][0]<<","<<b1<<";"<<k2<<","<<b2<<";\n";

    		CvPoint2D32f temp_point = lines_cross_point(k1,b1,k2,b2);
    		cross_points_yellow[m*positive_k_total+n][0] = (int)temp_point.x;
    		cross_points_yellow[m*positive_k_total+n][1] = (int)temp_point.y;
    		//cout<<"Point("<<m<<","<<m<<")=("<<cross_points_yellow[m*positive_k_total+n][0]<<","<<cross_points_yellow[m*positive_k_total+n][1]<<")\n";
    	}

    }

    /***Find Useful Points***/
    int alfa_point[2];
    int beta_point[2];
    int gamma_point[2];
    int theta_point[2];
    int width_point_delt_x = 0;
    int width_point_delt_y = 0;
    int length_point_delt_x = 0;
    int length_point_delt_y = 0;
    float delt_x_scale = 0.2f;
    float delt_y_scale = 0.2f;
    float dist_threshold = 40.f;

    int cross_points_position_right_enemy[5][5][2] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0}; //(image_x, image_y)
    int cross_points_position_left_enemy[5][5][2] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0}; //(image_x, image_y)

    if(!camera_left_side) //right, enemy
    {
    	alfa_point[0] = cross_points_yellow[0][0];
    	alfa_point[1] = cross_points_yellow[0][1];
    	beta_point[0] = cross_points_yellow[positive_k_total][0];
    	beta_point[1] = cross_points_yellow[positive_k_total][1];
    	gamma_point[0] = cross_points_yellow[1][0];
    	gamma_point[1] = cross_points_yellow[1][1];
    	theta_point[0] = cross_points_yellow[positive_k_total+1][0];
    	theta_point[1] = cross_points_yellow[positive_k_total+1][1];
  	
    	//first line 
    	width_point_delt_x = gamma_point[0] - alfa_point[0];
    	width_point_delt_y = gamma_point[1] - alfa_point[1];

		cross_points_position_right_enemy[0][0][0] = alfa_point[0];
		cross_points_position_right_enemy[0][0][1] = alfa_point[1];    	
    	for(int i = 1; i < 5; i++)
    	{
    		//predict position
    		int predict_x = cross_points_position_right_enemy[0][i-1][0] + (int)(width_point_delt_x * (1 + delt_x_scale));
    		int predict_y = cross_points_position_right_enemy[0][i-1][1] + (int)(width_point_delt_y * (1 + delt_y_scale));
    		dist_threshold = (abs(width_point_delt_x) + abs(width_point_delt_y))/5.f;
    		int near_points_counter = 0;
    		for(int j = 0; j < max_points_number; j++)
    		{
    			if(point_distance(predict_x, predict_y, cross_points_yellow[j][0], cross_points_yellow[j][1]) < dist_threshold)
    			{
    				cross_points_position_right_enemy[0][i][0] += cross_points_yellow[j][0];
    				cross_points_position_right_enemy[0][i][1] += cross_points_yellow[j][1];
    				near_points_counter ++;
    			}
    		}
    		if(near_points_counter > 0)
    		{
    			cross_points_position_right_enemy[0][i][0] = cross_points_position_right_enemy[0][i][0]/near_points_counter;
    			cross_points_position_right_enemy[0][i][1] = cross_points_position_right_enemy[0][i][1]/near_points_counter;	
    		}
    		else
    		{
    			cross_points_position_right_enemy[0][i][0] = predict_x;
    			cross_points_position_right_enemy[0][i][1] = predict_y;
    		}	

    		width_point_delt_x = cross_points_position_right_enemy[0][i][0] - cross_points_position_right_enemy[0][i-1][0];
    		width_point_delt_y = cross_points_position_right_enemy[0][i][1] - cross_points_position_right_enemy[0][i-1][1];
    		//cout<<"a time \n";
    	}

    	//second line 
    	width_point_delt_x = theta_point[0] - beta_point[0];
    	width_point_delt_y = theta_point[1] - beta_point[1];

    	cross_points_position_right_enemy[1][0][0] = beta_point[0];
		cross_points_position_right_enemy[1][0][1] = beta_point[1];    	
    	for(int i = 1; i < 5; i++)
    	{
    		//predict position
    		int predict_x = cross_points_position_right_enemy[1][i-1][0] + (int)(width_point_delt_x * (1 + delt_x_scale));
    		int predict_y = cross_points_position_right_enemy[1][i-1][1] + (int)(width_point_delt_y * (1 + delt_y_scale));
    		dist_threshold = (abs(width_point_delt_x) + abs(width_point_delt_y))/5.f;
    		int near_points_counter = 0;
    		for(int j = 0; j < max_points_number; j++)
    		{
    			if(point_distance(predict_x, predict_y, cross_points_yellow[j][0], cross_points_yellow[j][1]) < dist_threshold)
    			{
    				cross_points_position_right_enemy[1][i][0] += cross_points_yellow[j][0];
    				cross_points_position_right_enemy[1][i][1] += cross_points_yellow[j][1];
    				near_points_counter ++;
    			}
    		}
    		if(near_points_counter > 0)
    		{
    			cross_points_position_right_enemy[1][i][0] = cross_points_position_right_enemy[1][i][0]/near_points_counter;
    			cross_points_position_right_enemy[1][i][1] = cross_points_position_right_enemy[1][i][1]/near_points_counter;	
    		}
    		else
    		{
    			cross_points_position_right_enemy[1][i][0] = predict_x;
    			cross_points_position_right_enemy[1][i][1] = predict_y;
    		}	

    		width_point_delt_x = cross_points_position_right_enemy[1][i][0] - cross_points_position_right_enemy[1][i-1][0];
    		width_point_delt_y = cross_points_position_right_enemy[1][i][1] - cross_points_position_right_enemy[1][i-1][1];
    		//cout<<"a time \n";
    	}
        
        
    	//other lines
    	for(int j = 0; j < 5; j++)
    	{
    		length_point_delt_x = cross_points_position_right_enemy[1][j][0]- cross_points_position_right_enemy[0][j][0];
    		length_point_delt_y = cross_points_position_right_enemy[1][j][1]- cross_points_position_right_enemy[0][j][1];
            //cout<<"length_point_delt="<<length_point_delt_x<<","<<length_point_delt_y<<" * ";
    		
    		for(int i = 2; i < 5; i++)
	    	{
	    		int predict_x = cross_points_position_right_enemy[i-1][j][0] + (int)(length_point_delt_x * (1 + delt_x_scale));
	    		int predict_y = cross_points_position_right_enemy[i-1][j][1] + (int)(length_point_delt_y * (1 + delt_y_scale));
	    		//cout<<"("<<i<<","<<j<<")=("<<predict_x<<","<<predict_y<<")    ";

	    		dist_threshold = (abs(length_point_delt_x) + abs(length_point_delt_y))/3.f;
	    		int near_points_counter = 0;
	    		for(int m = 0; m < max_points_number; m++)
	    		{
	    			if(point_distance(predict_x, predict_y, cross_points_yellow[m][0], cross_points_yellow[m][1]) < dist_threshold)
	    			{
	    				cross_points_position_right_enemy[i][j][0] += cross_points_yellow[m][0];
	    				cross_points_position_right_enemy[i][j][1] += cross_points_yellow[m][1];
	    				near_points_counter ++;
	    			}
	    		}
	    		if(near_points_counter > 0)
	    		{
	    			cross_points_position_right_enemy[i][j][0] = cross_points_position_right_enemy[i][j][0]/near_points_counter;
	    			cross_points_position_right_enemy[i][j][1] = cross_points_position_right_enemy[i][j][1]/near_points_counter;	
	    		}
	    		else
	    		{
	    			cross_points_position_right_enemy[i][j][0] = predict_x;
	    			cross_points_position_right_enemy[i][j][1] = predict_y;
	    		}	

	    		length_point_delt_x = cross_points_position_right_enemy[i][j][0] - cross_points_position_right_enemy[i-1][j][0];
	    		length_point_delt_y = cross_points_position_right_enemy[i][j][1] - cross_points_position_right_enemy[i-1][j][1];
	    	    //cout<<"length_point_delt="<<length_point_delt_x<<","<<length_point_delt_y<<" # ";
	    	}
    	}

    	
    	/***Calculate real position ***/
    	if(robot_image_p[0] > 0.001) //image_position
    	{	
	    	//rank crosspoints by distance to robot, from small to large
	    	float distance_temp[5][5];
	    	for(int i = 0; i < 5; i++)
	    	{
	    		for(int j = 0; j < 5; j++)
	    		{
	    			distance_temp[i][j] = point_distance_f(robot_image_p[0], robot_image_p[1], cross_points_position_right_enemy[i][j][0], cross_points_position_right_enemy[i][j][1]);
	    		}
	    	}
	    	//nearest point
	    	float min_distance = 10000.f;
	    	int min_row, min_col;
	    	for(int i = 0; i < 5; i++)
	    	{
	    		for(int j = 0; j < 5; j++)
	    		{
	    			if(distance_temp[i][j] < min_distance)
	    			{
	    				min_distance = distance_temp[i][j];
	    				min_row = i;
	    				min_col = j;
	    			}
	    		}
	    	}
	    	//second nearest point with different row and col
	    	float min_distance_2 = 10000.f;
	    	int min_row_2, min_col_2;
	    	for(int i = 0; i < 5; i++)
	    	{
	    		for(int j = 0; j < 5; j++)
	    		{
	    			if(distance_temp[i][j] < min_distance_2 && distance_temp[i][j] > min_distance && i!= min_row && j!=min_col)
	    			{
	    				min_distance_2 = distance_temp[i][j];
	    				min_row_2 = i;
	    				min_col_2 = j;
	    			}
	    		}
	    	}


	    	//when right side
	    	CvPoint2D32f min_dist_p1_tr = point_slant_coordinate_tranlate(positive_k_average, negative_k_average, cross_points_position_right_enemy[min_row][min_col][0], cross_points_position_right_enemy[min_row][min_col][1]);
	    	CvPoint2D32f min_dist_p2_tr = point_slant_coordinate_tranlate(positive_k_average, negative_k_average, cross_points_position_right_enemy[min_row_2][min_col_2][0], cross_points_position_right_enemy[min_row_2][min_col_2][1]);
	    	CvPoint2D32f robot_image_p_tr = point_slant_coordinate_tranlate(positive_k_average, negative_k_average, robot_image_p[0], robot_image_p[1]);

	    	float kx = (cross_points_real_position_right_enemy[min_row][min_col][0] - cross_points_real_position_right_enemy[min_row_2][min_col_2][0]) / (min_dist_p1_tr.x - min_dist_p2_tr.x);
	    	float ky = (cross_points_real_position_right_enemy[min_row][min_col][1] - cross_points_real_position_right_enemy[min_row_2][min_col_2][1]) / (min_dist_p1_tr.y - min_dist_p2_tr.y);
	    	
	    	robot_real_p[0] = (cross_points_real_position_right_enemy[min_row][min_col][0] + kx*(robot_image_p_tr.x - min_dist_p1_tr.x) + cross_points_real_position_right_enemy[min_row_2][min_col_2][0] + kx*(robot_image_p_tr.x - min_dist_p2_tr.x))/2.f;
			robot_real_p[1] = (cross_points_real_position_right_enemy[min_row][min_col][1] + ky*(robot_image_p_tr.y - min_dist_p1_tr.y) + cross_points_real_position_right_enemy[min_row_2][min_col_2][1] + ky*(robot_image_p_tr.y - min_dist_p2_tr.y))/2.f;

	    	//position calculate
	    	//cout<<"\n &"<<min_row<<" & "<< min_col << endl;	
	    	//cout<<"\n &"<<min_row_2<<" & "<< min_col_2 << endl;	

	    	//cout<<"Point1 Position = ("<<cross_points_real_position_right_enemy[min_row][min_col][0]<<","<<cross_points_real_position_right_enemy[min_row][min_col][1]<<")\n";
	    	//cout<<"Point2 Position = ("<<cross_points_real_position_right_enemy[min_row_2][min_col_2][0]<<","<<cross_points_real_position_right_enemy[min_row_2][min_col_2][1]<<")\n";
            cout<<"Robot Real Position = ("<<robot_real_p[0]<<","<<robot_real_p[1]<<")\n";
            cvCircle(color_yellow, cvPoint(cross_points_position_right_enemy[min_row][min_col][0], image_height - cross_points_position_right_enemy[min_row][min_col][1]), 4, CV_GREEN, 6);
            cvCircle(color_yellow, cvPoint(cross_points_position_right_enemy[min_row_2][min_col_2][0], image_height - cross_points_position_right_enemy[min_row_2][min_col_2][1]), 4, CV_GREEN, 6);
    	}
    	else 
    	{
    		cout<<"Can not find robot!!"<<endl;
    		robot_real_p[0] = -1000.f;
    		robot_real_p[1] = -1000.f;
    	}
    	
    		
    }



    /***Draw cross points***/

    for(int i = 0; i < 5; i++)
    {
    	for(int j = 0; j < 5; j++)
    	{
    		int x = cross_points_position_right_enemy[i][j][0];
    		int y = image_height - cross_points_position_right_enemy[i][j][1];
    		cvCircle(color_yellow, cvPoint(x, y), 10, CV_GREEN, 2);
    	}
    	
    }
    cvCircle(color_yellow, cvPoint((int)robot_image_p[0], image_height - (int)robot_image_p[1]), 10, CV_RED, 2);

    


	cvNamedWindow("image_raw");
	cvShowImage("image_raw",image_raw);
	cvNamedWindow("fill_color");
	cvShowImage("fill_color",fill_color);

	cvNamedWindow("color_blue");
	cvShowImage("color_blue",color_blue);
	cvNamedWindow("color_yellow");
	cvShowImage("color_yellow",color_yellow);
    
    cvWaitKey(0);

    cvDestroyWindow("image_raw");
    cvReleaseImage(&image_raw);
    cvDestroyWindow("fill_color");
    cvReleaseImage(&fill_color);

    cvReleaseImage(&blue);
    cvReleaseImage(&blue_canny);
    cvDestroyWindow("color_blue");
    cvReleaseImage(&color_blue);

    cvReleaseImage(&yellow);
    cvReleaseImage(&yellow_canny);
    cvDestroyWindow("color_yellow");
    cvReleaseImage(&color_yellow);

    cvReleaseImage(&red);

    cvReleaseMat(&blue_lines_mat);
    cvReleaseMat(&yellow_lines_mat);

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


void draw_result_lines(CvMat *lines_mat, IplImage* image, CvScalar color, int thickness)
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
    	cvLine(image, p1, p2 , color, thickness, CV_AA, 0);
    }
}

CvPoint2D32f point_slant_coordinate_tranlate(float kx, float ky, float x0, float y0)
{
	CvPoint2D32f x_crossp = lines_cross_point(ky, y0-ky*x0, kx, 0.f);
	CvPoint2D32f y_crossp = lines_cross_point(kx, y0-kx*x0, ky, 0.f);
	CvPoint2D32f result_p;
	result_p.x = point_distance_f(x_crossp.x, x_crossp.y, 0.f, 0.f);
	result_p.y = point_distance_f(y_crossp.x, y_crossp.y, 0.f, 0.f);

	if((x0 - y0 / ky) < 0) result_p.x = 0.f - result_p.x;
	if((x0 - y0 / kx) < 0) result_p.y = 0.f - result_p.y;

	return result_p;
}