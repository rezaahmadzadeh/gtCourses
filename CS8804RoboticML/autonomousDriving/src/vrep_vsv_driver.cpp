#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
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

typedef std::pair<double, double> Coord;
typedef std::pair<double,int> Val;
typedef std::pair<Coord, Val> Mine;

class Demining {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub;
        ros::Subscriber kinect_sub;
        ros::Subscriber metal_sub;

        //arm interface publisher for rviz
        ros::Publisher marker_laserInterface_;
        ros::Publisher marker_normalTangente;
        ros::Publisher marker_pointTangente;
        ros::Publisher marker_lineTangente;
        ros::Publisher metal_pub;

        //frame setting
        tf::TransformListener listener_;
        std::string world_frame_;
        std::string pan_frame_;
        std::string tilt_frame_;
        std::string ground_frame_;
        std::string base_frame_;
        std::string tool_frame_;

        //control pub
        ros::Publisher tpos_pub; //arm
        ros::Publisher truck_control_pub; //truck

        //world setting
        int CN; //Cell Numbers per line
        double dCell;
        int L; //size of obsevration 
        double max_range_;
        int world_size;

        //height map
        image_transport::Publisher height_pub;
        cv::Mat_<double> height_ref;
        cv::Mat_<cv::Vec3b> height_image;  
        double threshold; //height_threshold

        //contour control
        cv::Mat_<cv::Vec3b> filter_contour_image; 
        image_transport::Publisher filter_contour_pub;

        //contour map   
        cv::Mat contour_image, height_image_gray, canny_output, threshold_image;
        double contour_height;
        std::vector<std::vector<cv::Point> > contours;        
        std::vector<cv::Vec4i> hierarchy;
        cv::RNG rng;
        image_transport::Publisher contour_pub;
        int L_contour; //size of obsevration 

        //contour processing
        pcl::PointCloud<pcl::PointXYZ> line;
        std::multimap<double,pcl::PointXYZ> distance_map;

        //control parameter
        double y_laser;
        double y_stat;
        int y_laser_input;
        double velocity;
        double k1, k2;
        double theta_max;
        double distance_max;

        //tangente seeting
        double look_ahead_x;
        int min_points;
        int min_points_line;

        //ransac param
        double tolerance;
        int n_samples;

        //arm param
        double min_x;
        double max_x;
        double min_z;
        double max_z;        
        double min_y;
        double max_y;
        double delta;

        //mine detection
        std::deque<Mine> mine_xy;
        double mine_threshold;
        double mine_dist_threshold;

    protected:

        std::pair<Eigen::Vector2f, Eigen::Vector2f> line_ransac(pcl::PointCloud<pcl::PointXYZ> lastpc_) {
            Eigen::Vector2f X, R; X << 0.0, 0.0;
            int S = 0;
            int n = lastpc_.size();
            Eigen::Vector2f N;
            for (unsigned int i=0;i<(unsigned)n_samples;i++) {
                //chode 2 random points a 2d point
                size_t i_a = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                size_t i_b = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
                Eigen::Vector2f A; A << lastpc_[i_a].x, lastpc_[i_a].y; 
                Eigen::Vector2f B; B << lastpc_[i_b].x, lastpc_[i_b].y;

                //compute line
                Eigen::Vector2f AB; AB = B-A; AB = AB/AB.norm();
                N << -AB[1], AB[0];
                N = N/N.norm();

                int S_temp = 0;
                for(int i(0) ; i < n ; i++){
                    Eigen::Vector2f P; P << lastpc_[i].x, lastpc_[i].y;
                    double x = fabs((P-A).dot(N));
                    if (x<tolerance){
                        S_temp++;
                    }
                }

                if(S_temp>S){
                    X[0] = N[0]; //b
                    X[1] = N[1]; //a
                    R[0] = A[0];
                    R[1] = A[1];
                    S = S_temp;
                }
            }

            //normal vector visualization
            
            visualization_msgs::Marker a;
            a.header.frame_id = world_frame_;
            a.header.stamp = ros::Time();
            a.ns = "points_and_lines";
            a.action = visualization_msgs::Marker::ADD;
            a.id = 0;
            a.type = visualization_msgs::Marker::ARROW;
            a.scale.x = 0.5;
            a.scale.y = 1;
            a.color.r = 1.0f;
            a.color.a = 1.0;
            geometry_msgs::Point start, end;
            start.x = start.y = 0;
            end.x = X[0];
            end.y = X[1];
            a.points.push_back(start);
            a.points.push_back(end);
            marker_normalTangente.publish(a);
            

            //point on line visualization
            
            visualization_msgs::Marker r;
            r.header.frame_id = world_frame_;
            r.header.stamp = ros::Time();
            r.ns = "points_and_lines";
            r.action = visualization_msgs::Marker::ADD;
            r.id = 0;
            r.type = visualization_msgs::Marker::POINTS;
            r.scale.x = 1;
            r.scale.y = 1;
            r.color.g = 1.0f;
            r.color.a = 1.0;
            geometry_msgs::Point origine;
            origine.x = R[0];
            origine.y = R[1];
            r.points.push_back(origine);
            marker_pointTangente.publish(r);
                  
            return std::pair<Eigen::Vector2f, Eigen::Vector2f>(X,R); //normal to the tangente
        } 


        void control_callback(const sensor_msgs::PointCloud2ConstPtr msg){
            line.clear();
            distance_map.clear();

            /*height map defintion*/
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg,temp);
            pcl::PointCloud<pcl::PointXYZ> worldpc;
            listener_.waitForTransform(world_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(world_frame_,msg->header.stamp, temp, msg->header.frame_id,  worldpc, listener_);

            unsigned int n = temp.size();
            unsigned int i = 0;
            std::vector<size_t> pidx;
            for (i=0;i<n;i++) { //processing
                float d = temp[i].z; 
                // In the sensor frame, this point would be inside the camera
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                // Measure the point distance in the base frame
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }

            //height computation
            for(i=0;i<pidx.size();i++) { 
                pcl::PointXYZ pointTemp = worldpc[pidx[i]];
                float x = pointTemp.x + 25;
                float y = pointTemp.y + 25;
                float z = pointTemp.z;
                int l = ceil(x/dCell)-1;
                int c = ceil(y/dCell)-1;               

                if(!(l>=0)) continue;
                if(!(l<CN)) continue;
                if(!(c>=0)) continue;
                if(!(c<CN)) continue;
                if(height_ref(l,c,0)<L) {
                    height_ref(l,c,1) = (height_ref(l,c,1)*height_ref(l,c,0) + z)/(height_ref(l,c,0) + 1);
                    height_ref(l,c,0) = height_ref(l,c,0) + 1;
                }
            }

            //image computation
            for(size_t l =0; l< (unsigned int) CN; l++) {
                for(size_t c =0; c<(unsigned int)CN; c++) {
                    if(height_ref(l,c,0) ==0) {
                        height_image(l,c)[0] = 0;
                        height_image(l,c)[1] = 0;
                        height_image(l,c)[2] = 0;
                    }
                    else if(height_ref(l,c,1) > threshold) {
                        height_image(l,c)[0] = 255;
                        height_image(l,c)[1] = 255;
                        height_image(l,c)[2] = 255;
                        height_ref(l,c,2) = 1; //=1 if brown
                    }
                    else {
                        height_image(l,c)[0] = 127;
                        height_image(l,c)[1] = 127;
                        height_image(l,c)[2] = 127;
                    }
                }
            }


            //height map visualization
            sensor_msgs::ImagePtr height_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", height_image).toImageMsg();
            height_pub.publish(height_image_msg);  

            // contour definition
            cv::cvtColor(height_image, height_image_gray, CV_BGR2GRAY); 
            cv::threshold(height_image_gray, threshold_image, 127, 255, CV_THRESH_BINARY); 
            cv::Canny( height_image_gray, canny_output, 127, 255, 3 );
            cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

            // contour filtering to get only green/brozn interface
            std::vector<std::pair<int, int> > cidx; 
            for(size_t i=0; i<contours.size(); i++){
                for(size_t j=0; j<contours[i].size(); j++){
                    bool tmp = true;
                    cv::Point p = (contours[i])[j];
                    for(size_t l = p.x-1; l< (unsigned int) p.x+2; l++){
                        for(size_t k = p.y-1; k< (unsigned int) p.y+2; k++){  
                            if (tmp==false) {
                                continue;
                            }
                            if(!(l>=0)) continue;
                            if(!(l< (unsigned int)CN)) continue;
                            if(!(k>=0)) continue;
                            if(!(k< (unsigned int)CN)) continue;

                            if(height_ref(k,l,0) == 0) {
                                tmp = false;
                            }
                        }
                    }

                    if (tmp==true) {
                        cidx.push_back(std::pair<int,int>(i,j));
                    }
                }
            }

            //raw contour visualization
            cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
            for( size_t i = 0; i< contours.size(); i++ ) {
                cv::Scalar color = cv::Scalar( 0, 0, 255);
                cv::drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
            }
            sensor_msgs::ImagePtr contour_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
            contour_pub.publish(contour_image_msg);  

            // filtered contour visualization 
            for (size_t l =0; l<cidx.size(); l++) {
                int i = cidx[l].first;
                int j = cidx[l].second;
                int x = ((contours[i])[j]).x;
                int y = ((contours[i])[j]).y;
                filter_contour_image(x,y)[2] = 255;
            }
            sensor_msgs::ImagePtr filter_contour_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", filter_contour_image).toImageMsg();
            filter_contour_pub.publish(filter_contour_image_msg);  


            /* contour filtering to get only the relevant contour window*/

            //define lookahead point
            geometry_msgs::PointStamped look_ahead;
            look_ahead.header.stamp = msg->header.stamp;
            look_ahead.header.frame_id = base_frame_;
            look_ahead.point.x = look_ahead_x;
            look_ahead.point.y = 0;           
            look_ahead.point.z = 0;

            //contour sorting by distance
            for (size_t l =0; l<cidx.size(); l++) {
                int i = cidx[l].first;
                int j = cidx[l].second;

                int x = ((contours[i])[j]).x;
                int y = ((contours[i])[j]).y;

                //build point to store
                pcl::PointXYZ pcl_p;
                pcl_p.y = (x*dCell - world_size);
                pcl_p.x = (y*dCell - world_size);
                pcl_p.z = 0.0;

                //compute angle and distance for keyed storing
                geometry_msgs::PointStamped p_world, p_base;
                p_world.point.x =  pcl_p.x; //checked
                p_world.point.y = pcl_p.y;
                p_world.point.z = 0.0;
                p_world.header.stamp = msg->header.stamp;
                p_world.header.frame_id = world_frame_;
                listener_.waitForTransform(base_frame_, world_frame_,p_world.header.stamp,ros::Duration(1.0));
                listener_.transformPoint(base_frame_,p_world, p_base);

                float x_base = p_base.point.x;
                float y_base = p_base.point.y;
                double distance = sqrt((x_base - look_ahead.point.x) * (x_base - look_ahead.point.x) + y_base*y_base);

                //stored point is in world frame
                distance_map.insert(std::pair<double,pcl::PointXYZ>(distance,pcl_p));
            }
            
            visualization_msgs::Marker points;
            points.header.frame_id = world_frame_;
            points.header.stamp = msg->header.stamp;
            points.ns = "contour_window";
            points.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = 1.0;
            points.id = 0;
            points.type = visualization_msgs::Marker::POINTS;
            points.scale.x = 0.5;
            points.scale.y = 0.5;
            points.color.r = 1.0f;
            points.color.a = 1.0;

            // build contour window made of p_to_take points if you have enough point
            double p_to_take = min_points; 
            if(distance_map.size() < (unsigned int) min_points) {
                p_to_take = distance_map.size();
            }

            if(distance_map.size() == 0) {
                //if you have no points but have laser signal
                if (y_laser_input ==1) { 
                    y_laser_input = 0;
                    geometry_msgs::Twist t;
                    t.linear.x = velocity;
                    double y = y_stat-y_laser;
                  
                    t.angular.z = k1*y;
                    t.linear.y = t.linear.z = t.angular.x = t.angular.y =  0.0;
                    truck_control_pub.publish(t);
                }
                //if you have no points and no laser signal, go straight
                else { 
                    geometry_msgs::Twist t;
                    t.linear.x = velocity;
                    t.angular.z = 0.0;
                    t.linear.y = t.linear.z = t.angular.x = t.angular.y = 0.0;
                    truck_control_pub.publish(t);
                }
                y_laser_input=0;
            }

            else {
                std::map<double, pcl::PointXYZ>::iterator it = distance_map.begin();
                for (int i=0; i<p_to_take; i++) {
                    line.push_back(it->second);
                    geometry_msgs::Point mp;
                    mp.x = (it->second).x;
                    mp.y = (it->second).y;
                    mp.z = (it->second).z;
                    points.points.push_back(mp);
                    it++;
                }

                //contour window visualization
                marker_lineTangente.publish(points);             

                
                if(line.size() == (unsigned int) min_points) { 
                    //get a vector orthogonal to the tangente and one point of it
                    std::pair<Eigen::Vector2f,Eigen::Vector2f> NR = line_ransac(line); 
                    Eigen::Vector2f N = NR.first;
                    Eigen::Vector2f origin_line = NR.second; 

                    //compute coordinate of the look ahead points in the world
                    geometry_msgs::PointStamped la_world;
                    listener_.waitForTransform(world_frame_, base_frame_,msg->header.stamp,ros::Duration(1.0));
                    listener_.transformPoint(world_frame_,look_ahead, la_world);

                    //compute the ecart distance from base axis to line
                    Eigen::Vector2f B;
                    B << la_world.point.x, la_world.point.y;
                    double y = y_stat - fabs((B-origin_line).dot(N));

                    /*if (fabs(y)> distance_max) {
                        distance_max = fabs(y);
                        std::cout << "distance_max=" << distance_max << std::endl;
                    }*/
                
                    // compute angle difference computation
                    geometry_msgs::PointStamped rdrive_in, rdrive_out;
                    rdrive_in.point.x = 0.0;
                    rdrive_in.point.y = 0.0;
                    rdrive_in.header.stamp = msg->header.stamp;
                    rdrive_in.header.frame_id = "VSV/RRDrive";
                    listener_.waitForTransform("VSV/RLDrive", "VSV/RRDrive",msg->header.stamp,ros::Duration(1.0));
                    listener_.transformPoint("VSV/RLDrive",rdrive_in, rdrive_out);

                    Eigen::Vector2f V; //vector colinear to reer axis
                    V << rdrive_out.point.x, rdrive_out.point.y;
                    V = V/V.norm();

                    double theta_v = atan2(V(1),V(0));
                    double theta_n = atan2(N(1),N(0));
                    if(theta_n > M_PI) {
                        theta_n = theta_n - M_PI;
                    }
                    double teta = remainder(theta_n -theta_v, M_PI);

                    /*if (fabs(teta) > theta_max) {
                        theta_max = fabs(teta);
                        std::cout << "theta_max=" << theta_max << std::endl;
                    }*/

                    geometry_msgs::Twist t;
                    t.linear.x = velocity;
                    t.angular.z = k1*y + k2*teta;
                    t.linear.y = t.linear.z = t.angular.x = t.angular.y = 0.0;
                    truck_control_pub.publish(t);
                }
                //if you don't have enough point of contour
                else {
                    //if you have laser information
                    if (y_laser_input ==1) {
                        y_laser_input = 0;
                        geometry_msgs::Twist t;
                        t.linear.x = velocity;
                        double y = y_stat - y_laser;
                        t.angular.z = k1*y;
                        t.linear.y = t.linear.z = t.angular.x = t.angular.y =  0.0;
                        truck_control_pub.publish(t);
                    }
                    else {
                        geometry_msgs::Twist t;
                        t.linear.x = velocity;
                        t.angular.z = 0.0;
                        t.linear.y = t.linear.z = t.angular.x = t.angular.y = 0.0;
                        truck_control_pub.publish(t);
                    }
                }
                y_laser_input=0; //to acknowledge next time the laser gets y info
            }
            
        }


        void laser_callback(const sensor_msgs::PointCloud2ConstPtr msg){
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg,temp);  
            pcl::PointCloud<pcl::PointXYZ> worldpc;
            listener_.waitForTransform(world_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(world_frame_,msg->header.stamp, temp, msg->header.frame_id,  worldpc, listener_);

            pcl::PointCloud<pcl::PointXYZ> panpc;
            listener_.waitForTransform(pan_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(pan_frame_,msg->header.stamp, temp, msg->header.frame_id,  panpc, listener_);


            unsigned int n = temp.size();
            unsigned int i = 0;
            bool interface_found = false;
            for(i=0;i<n;i++) {
                float x = worldpc[i].x;
                float y = worldpc[i].y;
                float z = worldpc[i].z;

                geometry_msgs::Point tposMsg;
                tposMsg.x = panpc[i].x;
                tposMsg.y = panpc[i].y;
                tposMsg.z = panpc[i].z;

                if((z > min_z) && (z < max_z) && (tposMsg.y < min_y) && (!interface_found) ) {  

                    //laser_found interface visualization
                    interface_found = true;
                    visualization_msgs::Marker points;
                    points.header.frame_id = world_frame_;
                    points.header.stamp = ros::Time::now();
                    points.ns = "points";
                    points.action = visualization_msgs::Marker::ADD;
                    points.pose.orientation.w = 1.0;
                    points.id = 0;
                    points.type = visualization_msgs::Marker::POINTS;
                    points.scale.x = 0.01;
                    points.scale.y = 0.01;
                    points.color.g = 1.0f;
                    points.color.a = 1.0;
                    geometry_msgs::Point p;
                    p.x = x;
                    p.y = y;
                    p.z = z + delta;
                    points.points.push_back(p);                
                    marker_laserInterface_.publish(points); 

                    tpos_pub.publish(tposMsg);

                    //laser information storage for truck_control
                    geometry_msgs::PointStamped p_world, p_pan;
                    p_world.point.x = x; 
                    p_world.point.y = y;    
                    p_world.point.z = z;
                    p_world.header.stamp = msg->header.stamp;
                    p_world.header.frame_id = world_frame_;
                    listener_.waitForTransform(pan_frame_, world_frame_,p_world.header.stamp,ros::Duration(1.0));
                    listener_.transformPoint(pan_frame_,p_world, p_pan);
                    y_laser = -p_pan.point.y;
                    y_laser_input = 1;
                }
                i++;
            }
        }



        void metal_callback(std_msgs::Float32 msg){

            if(msg.data > (float) mine_threshold) {
                //compute mine coordinate
                geometry_msgs::PointStamped t_world, t_tool;
                t_tool.point.x = 0.0; 
                t_tool.point.y = 0.0;    
                t_tool.point.z = 0.0;
                t_tool.header.stamp = ros::Time();
                t_tool.header.frame_id = tool_frame_;
                listener_.waitForTransform(world_frame_, tool_frame_,ros::Time(),ros::Duration(1.0));
                listener_.transformPoint(world_frame_,t_tool, t_world);
                double x = t_world.point.x;
                double y = t_world.point.y;


                if (mine_xy.size() == 0) {
                    mine_xy.push_back(Mine(Coord(x,y),Val(msg.data,1)));
                }
                //compute the nearest mine from this points
                else {
                    double dist_min= 1000.0;
                    double x_min = 0.0;
                    double y_min = 0.0;
                    int idx_min = -1;

                    for (size_t i=0; i<mine_xy.size(); i++) {
                        double x_tmp = mine_xy[i].first.first;
                        double y_tmp = mine_xy[i].first.second;
                        double dist= sqrt((x - x_tmp)*(x - x_tmp) + (y - y_tmp)*(y-y_tmp));

                        if(dist < dist_min) {
                            x_min = x_tmp;
                            y_min = y_tmp;
                            dist_min = dist;
                            idx_min = i;
                        }
                    }            
                    //if there is no mine near this point, it is a new mine
                    if(dist_min > mine_dist_threshold) {
                        mine_xy.push_back(Mine(Coord(x,y),Val(msg.data,1)));
                    }
                    //compute a barycenter of the mine and the point
                    else if (idx_min>=0) {
                        double val = mine_xy[idx_min].second.first;
                        int num = mine_xy[idx_min].second.second;
                        mine_xy.erase(mine_xy.begin() +idx_min);

                        double x_new = (x*msg.data + x_min*val*num)/(num*val+msg.data);
                        double y_new = (y*msg.data + y_min*val*num)/(num*val+msg.data);
                        double val_new = (msg.data + num*val)/(num+1);
                        mine_xy.push_back(Mine(Coord(x_new,y_new), Val(val_new,(num+1))));
                    }

                }

                //mine visualization
                visualization_msgs::Marker metal_mark;
                metal_mark.header.frame_id = world_frame_;
                metal_mark.header.stamp = ros::Time();
                metal_mark.ns = "mine";
                metal_mark.action = visualization_msgs::Marker::ADD;
                metal_mark.pose.orientation.w = 1.0;
                metal_mark.id = 0;
                metal_mark.type = visualization_msgs::Marker::POINTS;
                metal_mark.scale.x = 1.0;
                metal_mark.scale.y = 1.0;
                metal_mark.color.b = 1.0f;
                metal_mark.color.a = 1.0;

                for (size_t i=0; i<mine_xy.size(); i++) {
                    geometry_msgs::Point p;
                    p.x = (mine_xy[i]).first.first;
                    p.y = (mine_xy[i]).first.second;
                    p.z = 0.0;
                    metal_mark.points.push_back(p);  
                }
                metal_pub.publish(metal_mark);
            }
        }


    public:
        Demining() : nh_("~") {

            //frame setting
            nh_.param("world_frame_",world_frame_, std::string("/world")); 
            nh_.param("pan_frame_",pan_frame_, std::string("/VSV/ArmPan")); 
            nh_.param("tilt_frame_",tilt_frame_, std::string("/VSV/ArmTilt")); 
            nh_.param("ground_frame_",ground_frame_, std::string("/VSV/ground")); 
            nh_.param("base_frame_",base_frame_, std::string("/VSV/base")); 
            nh_.param("tool_frame_",tool_frame_, std::string("/VSV/Tool")); 

            //interface setting
            nh_.param("min_z",min_z,0.5);
            nh_.param("max_z",max_z,1.0);            
            nh_.param("min_y",min_y,2.0);
            nh_.param("max_y",max_y,0.0);
            nh_.param("delta",delta,0.005);

            //world setting
            nh_.param("max_range_",max_range_,5.0);
            nh_.param("CN",CN,50);
            dCell = 50.0/double(CN);            
            nh_.param("L",L,10);
            nh_.param("threshold", threshold, 0.01);
            nh_.param("world_size", world_size, 25);

            //tangente setting
            nh_.param("min_points", min_points, 10);
            nh_.param("min_points_line", min_points_line, 10);            
            nh_.param("look_ahead_x" , look_ahead_x, 3.0);
            nh_.param("contour_height" , contour_height, 0.1);
            nh_.param("L_contour",L_contour,10);

            //ransac
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,1.0);

            //control param
            nh_.param("y_stat",y_stat,1.0);
            nh_.param("y_laser",y_laser,0.0);
            nh_.param("y_laser_input",y_laser_input,0);
            nh_.param("velocity",velocity,0.1);
            nh_.param("k1",k1,1.0);
            nh_.param("k2",k2,1.0);

            //mine detection
            nh_.param("mine_threshold", mine_threshold,0.8);
            nh_.param("mine_dist_threshold", mine_dist_threshold,1.0);    

            distance_max = 0.0;
            theta_max = 0.0;

            ros::Duration(0.5).sleep();
            kinect_sub = nh_.subscribe("/vrep/depthSensor",1, &Demining::control_callback,this);              
            scan_sub = nh_.subscribe("/vrep/hokuyoSensor",1, &Demining::laser_callback,this);
            metal_sub = nh_.subscribe("/vrep/metalDetector",1, &Demining::metal_callback,this);

            tpos_pub = nh_.advertise<geometry_msgs::Point>("target_pose",1);
            truck_control_pub = nh_.advertise<geometry_msgs::Twist>("control_truck",1);

            ros::NodeHandle(nh2);
            image_transport::ImageTransport image_handler(nh2);
            height_pub = image_handler.advertise("/vrep_vsv_driver/heightTopic",1);       
            contour_pub = image_handler.advertise("/vrep_vsv_driver/contourTopic",1);
            filter_contour_pub = image_handler.advertise("/vrep_vsv_driver/filterContourTopic",1);

            marker_laserInterface_ = nh_.advertise<visualization_msgs::Marker>("laser_interface_world",1);
            marker_normalTangente = nh_.advertise<visualization_msgs::Marker>("orth_vector",1);
            marker_pointTangente = nh_.advertise<visualization_msgs::Marker>("point",1);            
            marker_lineTangente = nh_.advertise<visualization_msgs::Marker>("line",1);
            metal_pub = nh_.advertise<visualization_msgs::Marker>("mine_pos",1);

            height_image = cv::Mat_<cv::Vec3b>(CN,CN,cv::Vec3b(0,0,0)); 
            filter_contour_image = cv::Mat_<cv::Vec3b>(CN,CN,cv::Vec3b(0,0,0));
 
            int sz[3] = {CN,CN,2};            
            height_ref = cv::Mat_<double>(3,sz); 
            height_ref = 0.0;
            rng(12345);

        }

};


int main(int argc, char* argv[]) {
    ros::init(argc,argv,"demining");
    Demining d;
    ros::spin();
}

