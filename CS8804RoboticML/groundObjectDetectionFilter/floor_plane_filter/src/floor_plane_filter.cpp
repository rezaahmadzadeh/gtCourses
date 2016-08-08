#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <ros/topic.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <string>
#include <stdio.h>
#include <fstream>



typedef std::pair<int,int> MatIndex;
typedef std::list<double> ZList;
typedef std::map<MatIndex,ZList> ZLStorage; //map of altitude list
typedef std::map<MatIndex,Eigen::VectorXd> M; //map of altitude matrix i.e. observation
typedef std::map<MatIndex,double> P; //P for parameters


class FloorPlaneFilter{
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub;
        tf::TransformListener listener_;
        image_transport::Publisher height_pub;
        image_transport::Publisher confidence_pub;
        image_transport::Publisher error_pub;

        std::string base_frame_;
        std::string world_frame_;

        int CN; //Cell Numbers per line
        double dCell;
        double max_range_;
        int L; //size of obsevration
        double up_bound;
        double low_bound;
        int maxObSize; //necessaire ? 
        int minObSize; //mnecessaire ?
        double sigma_min;
        double sigma_max;
        double error;

        P mu; 
        P sigma;
        M K; //stores the transpose of K
        M Z; //stores the altitude as eigen matrix
        ZLStorage zls; //stores the altitude of points sent by vrep

        double a;
        double r;
        double q; //value of the variance in the matrix covariance Q
        double bel0;

        cv::Mat_<int> kalmanDone;
        cv::Mat_<double> height_ref;
        cv::Mat_<double> height_error;

        cv::Mat_<cv::Vec3b> height_image;
        cv::Mat_<cv::Vec3b> confidence_image;
        cv::Mat_<cv::Vec3b> error_image;


    protected: 

        Eigen::MatrixXd list_to_eigen(ZList zl) {
            int n = zl.size();
            Eigen::VectorXd res(n);
            int i = 0;
            for (ZList::const_iterator it=zl.begin();it!=zl.end();it++) {
                res(i) = *it;
                i++;
            }
            return res;
        }
           
        void kalman(int l, int c) { //run kalman on the cell (l,c)
            int n = Z[MatIndex(l,c)].rows(); //numer of observation
            double mu_temp = mu[MatIndex(l,c)];
            double sigma_temp = sigma[MatIndex(l,c)];
            Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n,n) * q;
            Eigen::VectorXd C = Eigen::VectorXd::Constant(n,1.0);
            //Prediction step
            if (kalmanDone(l,c)==0) {
		        mu_temp =bel0*a;
		        kalmanDone(l,c) = 1;
            }
            else {
                mu_temp *=a;
                //kalmanDone(l,c) = 1;
            }
            sigma_temp = sigma_temp*a*a + r;
            K[MatIndex(l,c)] = (sigma_temp * (C.transpose()) * (C * sigma_temp * C.transpose() + Q).inverse() ).transpose();
            
            //Measurement update step
            mu[MatIndex(l,c)] = mu_temp + (K[MatIndex(l,c)].dot(Z[MatIndex(l,c)] - C*mu_temp));
            sigma[MatIndex(l,c)] = (1 - ((K[MatIndex(l,c)]).dot(C)) ) * sigma_temp;

            /*
             //sigma SETTING 
          	if ( sigma[MatIndex(l,c)] < sigma_min) { 
          		sigma_min = sigma[MatIndex(l,c)]; 
          	} 
            else if ( sigma[MatIndex(l,c)] > sigma_max) {
            	sigma_max = sigma[MatIndex(l,c)]; 
            }
            */
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {

            //TIME SETTING 
             /* struct timeval tbegin,tend;
             * double texec=0.;
             * gettimeofday(&tbegin,NULL);
             */

            // L SETTING
             /* std::ofstream fp;
             * fp.open("../catkin_ws/L.csv", std::ios::app);
             */ 

            // q SETTING 
             /*
             double q_average = 0.0;
             double q_var = 0.0;
             std::ofstream fp;
             fp.open("../catkin_ws/src/floor_plane_filter/q.csv", std::ios::app);
             */

            // Process point cloud to get rid of outlier and store indices of relevant points in pidx
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg,temp);

            pcl::PointCloud<pcl::PointXYZ> lastpc_;
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            pcl::PointCloud<pcl::PointXYZ> worldpc;
            listener_.waitForTransform(world_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(world_frame_,msg->header.stamp, temp, msg->header.frame_id,  worldpc, listener_);

            unsigned int n = temp.size();
            unsigned int i = 0;
            std::vector<size_t> pidx;
            for (i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);
                // In the sensor frame, this point would be inside the camera
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                // Measure the point distance in the base frame
                x = lastpc_[i].x;
                y = lastpc_[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }

                float z = lastpc_[i].z;
                if (z > up_bound) {
                    //to high
                    continue;
                }
                // If we reach this stage, we have an acceptable point, so
                // let's store it
                pidx.push_back(i);
            }

            //Distribute points
            for(i=0;i<pidx.size();i++) {
                pcl::PointXYZ pointTemp = worldpc[pidx[i]];
                float x = pointTemp.x + 5;
                float y = pointTemp.y + 5;
                float z = pointTemp.z;
                int l = ceil(x/dCell)-1;
                int c = ceil(y/dCell)-1;
                if(!(l>=0)) continue;
                if(!(l<CN)) continue;
                if(!(c>=0)) continue;
                if(!(c<CN)) continue;
                //limit the size of the observation set 
                if(zls[MatIndex(l,c)].size()< (unsigned int)L) {
                    zls[MatIndex(l,c)].push_back(z);
                    height_ref(l,c,1) +=z;
                    height_ref(l,c,0) +=1;
                }

            }

            //Conver zlist into matrix and run kalman algorithm with the new observations
            for(ZLStorage::const_iterator it = zls.begin(); it!=zls.end(); it++) {
                //put a minorant L on the data measurement size to run kalman filter
                int l = (it->first).first;
                int c = (it->first).second;
                if(zls[MatIndex(l,c)].size() == (unsigned int)L) {
                    Z[MatIndex(l,c)] = list_to_eigen(zls[MatIndex(l,c)]);
                    kalman(l,c);
                    zls[MatIndex(l,c)].clear();
                    height_error(l,c) = fabs(mu[MatIndex(l,c)] - height_ref(l,c,1)/height_ref(l,c,0));
                }
            }

            //Update images with new estimation results
            for(M::const_iterator it = Z.begin();it!=Z.end();it++) {
                int l = (it->first).first;
                int c = (it->first).second;
                double mu_tmp = mu[MatIndex(l,c)];
                if (mu_tmp < 0 ) {
                    mu_tmp =1 + mu_tmp;
                    int height_tmp =  (int)(mu_tmp*255);
                    height_image(l,c)[1] =  height_tmp %255;
                }
                else {
		            int height_tmp =  (int)(mu_tmp*255);
		            height_image(l,c)[2] = height_tmp % 255;
		            height_image(l,c)[0] = 255 -  height_image(l,c)[2];
		        }

                confidence_image(l,c)[2] = 255 - ((int)(sigma_max - sigma[MatIndex(l,c)]*1e9) %255);
                confidence_image(l,c)[0] = 255 - confidence_image(l,c)[2];

                error_image(l,c)[2] = ((int)(height_error(l,c) *100* 255)) %255;
            }

            //ERROR MEASUREMENT
			/*
            int compteur = 0;
            int l,c;

            for(l=0;l<CN;l++) {
                for(c=0;c<CN;c++) {
                    if(kalmanDone(l,c)==1) {
                        error = error + height_error(l,c);
                        compteur++;
                        error = error / compteur;
                        std::ofstream fp;
                        fp.open("../catkin_ws/error.csv");
                        fp << error << "\n";
                        fp.close();
                        error = 0.0;
                    }
                }
            }*/


            sensor_msgs::ImagePtr height_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", height_image).toImageMsg();
            height_pub.publish(height_image_msg);
            sensor_msgs::ImagePtr confidence_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", confidence_image).toImageMsg();
            confidence_pub.publish(confidence_image_msg);
            sensor_msgs::ImagePtr error_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", error_image).toImageMsg();
            error_pub.publish(error_image_msg);

            
            //sigma measurement 
            /*
             std::ofstream fp;
             fp.open("../catkin_ws/sigma.csv");
             fp << sigma_min << ',' << sigma_max << "\n";
             fp.close();
             */
             

            // q SETTING 
            /*
               for(i=0;i<pidx.size();i++) {
               pcl::PointXYZ pointTemp = worldpc[pidx[i]];
               float z = pointTemp.z;
               q_average += z;
               }
               q_average /= pidx.size();
               for(i=0;i<pidx.size();i++) {
               pcl::PointXYZ pointTemp = worldpc[pidx[i]];
               float z = pointTemp.z;
               q_var += (q_average - z)*(q_average - z);
               }
               q_var /= pidx.size();
               fp << q_average << ',' << q_var << "\n";
               fp.close();
               */

            // TIME SETTING
             /* gettimeofday(&tend,NULL);
              texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
              */

            // L SETTING 
             /* 
              fp << texec << "\n";
              fp.close();
             */

        }

    public:
        FloorPlaneFilter() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/bubbleRob"));
            nh_.param("world_frame",world_frame_, std::string("/world"));            
            nh_.param("max_range_",max_range_,5.0);
            nh_.param("CN",CN,20);
            nh_.param("L",L,10);
            nh_.param("up_bound",up_bound,0.55);
            nh_.param("low_bound",low_bound,-0.1);
            nh_.param("maxObSize",maxObSize,100);
            nh_.param("minObSize",minObSize,100);          
            nh_.param("a",a,1.0);
            nh_.param("r",r,1.0);
            nh_.param("q",q,1.0);
            nh_.param("bel0",bel0,0.5);
            nh_.param("sigma_max",sigma_max,685754.0);
            nh_.param("sigma_min",sigma_min,685722.0);
            dCell = 10.0/double(CN);
            error = 0.0;
            
            ros::Duration(1).sleep();
            ros::NodeHandle(nh2);
            image_transport::ImageTransport image_handler(nh2);
            height_pub = image_handler.advertise("/floor_plane_filter/heightTopic",1);
            confidence_pub =  image_handler.advertise("/floor_plane_filter/confidenceTopic",1);
            error_pub =  image_handler.advertise("/floor_plane_filter/errorTopic",1);
            scan_sub = nh_.subscribe("scans",1,&FloorPlaneFilter::pc_callback,this);
            
            kalmanDone = cv::Mat_<int>(CN,CN); kalmanDone = 0;
            height_error = cv::Mat_<double>(CN,CN); kalmanDone = 0.0;
            int sz[3] = {CN,CN,2};
            height_ref = cv::Mat_<double>(3,sz); height_ref = 0.0;
            height_image = cv::Mat_<cv::Vec3b>(CN,CN,cv::Vec3b(0,0,0));
            confidence_image = cv::Mat_<cv::Vec3b>(CN,CN,cv::Vec3b(0,0,0));
            error_image = cv::Mat_<cv::Vec3b>(CN,CN,cv::Vec3b(0,0,0));
      
        }

};


int main(int argc, char* argv[]) {
    ros::init(argc,argv,"floor_plane_filter");

    FloorPlaneFilter fp;

    ros::spin();


}
















