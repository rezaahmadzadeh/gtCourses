#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>


class FloorPlaneRansac {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        ros::Publisher data_pub;

        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        double tolerance;
        int n_samples; 

        pcl::PointCloud<pcl::PointXYZ> lastpc_;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            
            // Timer
            struct timeval tbegin,tend;
            double texec=0.;
            gettimeofday(&tbegin,NULL);
			
			// Receive the point cloud and convert it to the right format
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            unsigned int n = temp.size();
            std::vector<size_t> pidx; // Indices of useful points i.e. points 
            // that are neither too near nor too far
            for (unsigned int i=0;i<n;i++) {
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
                // If we reach this stage, we have an acceptable point, so
                // let's store its index
                pidx.push_back(i);
            }
            
            // Finding planes: z = a*x + b*y + c
            // n: number of useful point in the point cloud
            n = pidx.size();
            size_t best = 0;
            double X[3] = {0,0,0};
	    	int S(0);
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            for (unsigned int i=0;i<(unsigned)n_samples;i++) {
                // Select a random number in [0,i-1]
                size_t j = std::min((rand() / (double)RAND_MAX) * i,(double)i-1);
                Eigen::Vector3f P; P << lastpc_[pidx[0]].x, lastpc_[pidx[0]].y, lastpc_[pidx[0]].z; 
                double x = P.dot(P);
                Eigen::Vector3f Q = P.cross(P); 
                double norm = P.norm();

				//Choose 3 random points of indices i_a, i_b, i_c
				size_t i_a = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				size_t i_b = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);
				size_t i_c = std::min((rand() / (double)RAND_MAX) * n,(double)n-1);

				//Create points A,B,C
				Eigen::Vector3f A; A << lastpc_[pidx[i_a]].x, lastpc_[pidx[i_a]].y, lastpc_[pidx[i_a]].z;
				Eigen::Vector3f B; B << lastpc_[pidx[i_b]].x, lastpc_[pidx[i_b]].y, lastpc_[pidx[i_b]].z;
				Eigen::Vector3f C; C << lastpc_[pidx[i_c]].x, lastpc_[pidx[i_c]].y, lastpc_[pidx[i_c]].z;
            
				//Compute plan 
				Eigen::Vector3f AB; AB = B-A; AB = AB/AB.norm();
				Eigen::Vector3f AC; AC = C-A; AC = AC/AC.norm();
				Eigen::Vector3f N = AB.cross(AC); 		

				//Norm the normal vector to make the computation easier
				N = N/N.norm();

				//Compute the number of consistent points i.e. points which distance to computed plan is lower than tolerance.
				int S_temp = 0;
				for(int i(0) ; i < n ; i++) {
					Eigen::Vector3f P; P << lastpc_[pidx[i]].x, lastpc_[pidx[i]].y, lastpc_[pidx[i]].z;
					double x = (P-A).dot(N);
					if (x<tolerance){
						S_temp++;
					}
				}

				if(S_temp>S) {
					X[2] =N[2] ;
				 	X[0] = N[0]/N[2];
				 	X[1] = N[1]/N[2];

					X[2] = -( A[2]*X[2] + A[1]*X[1] + A[0]*X[0]);
					S = S_temp;
				}

			}
			
            // At the end, we store the best plane estimate in X = {a,b,c}.

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
            m.color.r = 0.0;
            m.color.g = 0.0;
            m.color.b = 1.0;

            marker_pub_.publish(m);
            

             // Timer sample and publication
            gettimeofday(&tend,NULL);
            texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;

			geometry_msgs::Twist datamsg;
			datamsg.linear.x = texec;
			datamsg.angular.x = X[0];
			datamsg.angular.y = X[1];
			datamsg.angular.z = X[2];
			data_pub.publish(datamsg);
        }

    public:
        FloorPlaneRansac() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_samples",n_samples,1000);
            nh_.param("tolerance",tolerance,0.2);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneRansac::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);
             data_pub = nh_.advertise<geometry_msgs::Twist>("dataTopic", 20);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Ransac");
    FloorPlaneRansac fp;

    ros::spin();
    return 0;
}


