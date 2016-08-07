#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>


class FloorPlaneHough {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        ros::Publisher data_pub;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        cv::Mat_<uint32_t> accumulator;

        int n_a, n_b, n_c;
        double a_min, a_max, b_min, b_max, c_min, c_max;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
			// Timer
             struct timeval tbegin,tend;
             double texec=0.;
             gettimeofday(&tbegin,NULL);

			// Receive the point cloud and convert it to the right format
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Projects the point cloud into the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            //
            unsigned int n = temp.size();
            std::vector<size_t> pidx; // Indices of useful points i.e. points 
            // that are neither too near nor too far
            for (unsigned int i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);
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
                // If we reach this stage, we have an acceptable point, so
                // let's store its index
                pidx.push_back(i);
            }
            
            //
            // BEGIN TODO
            // Finding planes: z = a*x + b*y + c using the hough transform
            // We use use the a_min,a_max,n_a variables (resp. b, c).
            n = pidx.size();
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            // fill the accumulator with zeros
            accumulator = 0;
            double da= (a_max - a_min)/n_a; //discretization interval for the a axis 
            double  db= (b_max - b_min)/n_b; //discretization interval for the b axis 
            double  dc = (c_max - c_min)/n_c; //discretization interval for the c axis 

            int ia(0);
            int ib(0);
            int ic(0);

            for (unsigned int i=0;i<n;i++) {
                double x = lastpc_[pidx[i]].x;
                double y = lastpc_[pidx[i]].y;
                double z = lastpc_[pidx[i]].z;
                
                double pa, pb, pc;
                for (ia=0 ; ia<n_a ; ia++) {
                    for (ib=0; ib<n_b ; ib++) {
                        pa = a_min + ia*da;
                        pb = b_min + ib*db;
                        pc = z - pa*x - pb*y;
                        ic = round( (pc - c_min)/dc);
                        if ((ic < n_c)&&(ic>=0)){
                            accumulator(ia,ib,ic) +=1;
                        }

                     }
                }
                

            // Update the accumulator based on current point
            } 
            // at this point the accumulator is complete for all points x,y,z => max -> plan parameter

            double X[3] = {0,0,0};
            // Use the accumulator to find the best plane parameters and store
            // them in X (this will be used for display later)
            // X = {a,b,c}

            //Compute indices of accumulator maximum 
            int max_idx[3]; max_idx[0] = 0; max_idx[1]=0; max_idx[2]=0;
            double max_temp = 0.0;
            for (ia=0 ; ia<n_a ; ia++) {
                for (ib=0 ; ib<n_b ; ib++) {
                    for(ic=0 ; ic<n_c ; ic++) {
                        if(accumulator(ia,ib,ic)>max_temp){
                            max_idx[0]=ia;
                            max_idx[1]=ib;
                            max_idx[2]=ic;
                            max_temp = accumulator(ia,ib,ic);
                        }
                    }
                }
            }
            
            X[0] = a_min+da*max_idx[0];
            X[1] = b_min+db*max_idx[1];
            X[2] = c_min+dc*max_idx[2];

            // END OF TODO
            ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X[0],X[1],X[2]);

            Eigen::Vector3f O,u,v,w;
            w << X[0], X[1], -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X[0]+0.0*X[1]+X[2];
            u << 2.0, 0.0, 2.0*X[0]+0.0*X[1]+X[2];
            u -= O;
            u /= u.norm();
            v = w.cross(u);

            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            tf::Quaternion Q;
            R.getRotation(Q);
            
            visualization_msgs::Marker m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = base_frame_;
            m.ns = "floor_plane";
            m.id = 1;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = O(0);
            m.pose.position.y = O(1);
            m.pose.position.z = O(2);
            tf::quaternionTFToMsg(Q,m.pose.orientation);
            m.scale.x = 1.0;
            m.scale.y = 1.0;
            m.scale.z = 0.01;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 1.0;

            marker_pub_.publish(m);

            // timer sample
            gettimeofday(&tend,NULL);
            texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
		        
			geometry_msgs::Twist datamsg;
			datamsg.linear.x = texec;
			datamsg.linear.y = n_b;
			datamsg.linear.z = n_c;
			datamsg.angular.x = X[0];
			datamsg.angular.y = X[1];
			datamsg.angular.z = X[2];
			data_pub.publish(datamsg);
        }

    public:
        FloorPlaneHough() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_a",n_a,10);
            nh_.param("a_min",a_min,-1.0);
            nh_.param("a_max",a_max,+1.0);
            nh_.param("n_b",n_b,10);
            nh_.param("b_min",b_min,-1.0);
            nh_.param("b_max",b_max,+1.0);
            nh_.param("n_c",n_c,10);
            nh_.param("c_min",c_min,-1.0);
            nh_.param("c_max",c_max,+1.0);

            assert(n_a > 0);
            assert(n_b > 0);
            assert(n_c > 0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("a: %d value in [%f, %f]",n_a,a_min,a_max);
            ROS_INFO("b: %d value in [%f, %f]",n_b,b_min,b_max);
            ROS_INFO("c: %d value in [%f, %f]",n_c,c_min,c_max);

            // the accumulator is created here as a 3D matrix of size n_a x n_b x n_c
            int dims[3] = {n_a,n_b,n_c};
            accumulator = cv::Mat_<uint32_t>(3,dims);
            
            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneHough::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);
            data_pub = nh_.advertise<geometry_msgs::Twist>("dataTopic", 20);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Hough");
    FloorPlaneHough fp;

    ros::spin();
    return 0;
}


