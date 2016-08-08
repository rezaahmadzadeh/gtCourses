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


typedef std::pair<int,int> MatIndex;
typedef std::list<pcl::PointXYZ> PointList;
typedef std::map<MatIndex,PointList> PointStorage;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



class FloorPlaneMapping {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub; //get point cloud from vrep sensors
        ros::Subscriber regression_sub; //get the plane parameter computed by the regression node
        ros::Publisher point_pub; //send point cloud msg to the regression node
        tf::TransformListener listener_;
        image_transport::Publisher image_pub;

        std::string base_frame_;
        std::string world_frame_;
        int CN;//number of cells per line in the world
        double dCell;//length of a cell, redundancy to avoid declaring d in the callback
        double max_range_;
        double angle;//maximum angle of ground above which the cell is not traversable
        int numSamples; //number of samples that the robot samples through its vision
        int nb_min;
        int ransac_n_iteration;
        double ransac_tolerance;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;

        PointStorage PS;
        PointStorage::const_iterator it;

		double xz;//proba that Xt = Zt for X, Z;
		double xxt; //proba that Xt-1 = Xt;
		
        //plane 0 = {1,0} an estimation has been/has not yet been done
        //plane 1 : how many points it has used until now to estimate this cell
        //plane 2 : how many new points detected to update the cell
        //plane 3-4-5 : plane parameters (a,b,c) for this cell
        cv::Mat_<double> E; 
        cv::Mat_<cv::Vec3b> R;
        
        


    protected: 
        //store the estimated parameters received from the regression node in E
        //Ransac for first estimation
         Eigen::Vector3f ransac (PointCloud last_pc, int l, int c) {
		
            Eigen::Vector3f X;
            int S(0);
            int n = last_pc.size();
			if (n > nb_min) {
		        for (unsigned int i=0;i<(unsigned)ransac_n_iteration;i++) {
					size_t i_a = rand() % n;
					size_t i_b = rand() % n;
					size_t i_c = rand() % n;
				
					if (i_a==i_b ||i_a==i_c||i_b==i_c) {
							continue;
					}
		            //Create points A,B,C
		            Eigen::Vector3f A; A << lastpc_[i_a].x, lastpc_[i_a].y, lastpc_[i_a].z;
		            Eigen::Vector3f B; B << lastpc_[i_b].x, lastpc_[i_b].y, lastpc_[i_b].z;
		            Eigen::Vector3f C; C << lastpc_[i_c].x, lastpc_[i_c].y, lastpc_[i_c].z;

		            //Compute plan 
		            Eigen::Vector3f AB; 
		            AB << lastpc_[i_b].x-lastpc_[i_a].x, lastpc_[i_b].y-lastpc_[i_a].y, lastpc_[i_b].z-lastpc_[i_a].z;
		            AB = AB/AB.norm();
		            Eigen::Vector3f AC; 
		            AC << lastpc_[i_c].x-lastpc_[i_a].x, lastpc_[i_c].y-lastpc_[i_a].y, lastpc_[i_c].z-lastpc_[i_a].z; 
		            AC = AC/AC.norm();
		            
		            Eigen::Vector3f Zero;
		            Zero << 0.0,0.0,0.0;
		            if (AB.cross(AC) == Zero) {
							continue;
					} 
					Eigen::Vector3f N;
					N = AB.cross(AC);	
		            //norm the normal vector to the computed plan
		            N = (1/N.norm())*N;

		            //Compute the number of consistent points i.e. points which distance to computed plan is lower than tolerance.
		            int S_temp(0);
		            for(int j(0); j<n; j++) {
		                Eigen::Vector3f P; 
		                P << lastpc_[j].x, lastpc_[j].y, lastpc_[j].z;
		                Eigen::Vector3f Tampon;
		                Tampon << P.x()-A.x(), P.y()-A.y(), P.z()-A.z();
		                double x = N.dot(Tampon);
		                if (fabs(x)<ransac_tolerance){
		                    S_temp++;
		                }
		            }

		            if(S_temp>S) {
						X = N;
		                S = S_temp;
		            }

		        }
		        return X;
		    }
        	else {
				Eigen::Vector3f Z;
				Z << 0.0, 0.0, 0.0;
				return Z;
			}
		}
	
		Eigen::MatrixXf regression(PointCloud v) //Compute the number of consistent points i.e. points which distance to computed plan is lower than tolerance.{
			int n = v.size();
			Eigen::MatrixXf X;
            Eigen::MatrixXf A(n,3);
            Eigen::MatrixXf B(n,1);
            for (unsigned int i=0;i<(unsigned)n;i++) {
                // Assign x,y,z to the coordinates of the point we are
                // considering.
                double x = v[i].x;
                double y = v[i].y;
                double z = v[i].z;

                A(i,0) = x;
                A(i,1) = y;
                A(i,2) = 1;

                B(i,0) = z;
            }
            X = (A.transpose()*A).inverse() * A.transpose() * B;
			return X;
		}


        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
        	//Clean the last point cloud sample
            PS.clear();
            
            // Filter the point cloud
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg,temp);

            //projection to process outliers and store indices of relevant points in pidx
			listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
			pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            unsigned int n = temp.size();
            unsigned int i = 0;
            
            std::vector<size_t> pidx;
            // First count the useful points
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
                
                // If we reach this stage, we have an acceptable point, so
                // let's store it
                pidx.push_back(i);
            }


            // Point distribution 
            pcl::PointCloud<pcl::PointXYZ> worldpc;
            //projection to compute the world cell of the relevant. Store result in worldpc
            listener_.waitForTransform(world_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(world_frame_,msg->header.stamp, temp, msg->header.frame_id,  worldpc, listener_);
            
            //adjust them to the matrix coordonnates
            for (i=0;i<pidx.size();i++) {
                
                worldpc[pidx[i]].x = worldpc[pidx[i]].x + 5;
                worldpc[pidx[i]].y = worldpc[pidx[i]].y + 5;
                int l = ceil(worldpc[pidx[i]].x/dCell)-1;
                int c = ceil(worldpc[pidx[i]].y/dCell)-1;

                if(!(l>=0)) continue;
                if(!(l<CN)) continue;
                if(!(c>=0)) continue;
                if(!(c<CN)) continue;
                
                PS[MatIndex(l,c)].push_back(worldpc[pidx[i]]);  
               
            }

            
            // Plane estimation for each cell
            for ( it=PS.begin();it!=PS.end();it++) {
                //list of points of the (l,c) cell
                PointList mylist = it->second;
                int l = (it->first).first;
                int c = (it->first).second;

                // Regression part
                PointCloud point_temp;
                for (PointList::const_iterator lit=mylist.begin() ; lit!=mylist.end() ; lit++) {
                    const pcl::PointXYZ & P = *lit;
                    point_temp.push_back(pcl::PointXYZ(P.x , P.y , P.z));
                }
                if (point_temp.size() < (unsigned)nb_min) {
						continue;
				}
                    
				Eigen::MatrixXf X = Eigen::MatrixXf::Zero(3, 1);
				X = regression(point_temp);
				point_temp.clear();
                int t = 0;
                if((X(0)<angle) && (X(1)<angle)) {
					t = 1;
				}
				
			    double xz_p (xz);
			    
			    // Bayesien treatment
			    double p, q;
			    //p = P(cell is traversable)
			    if (t == 1) {
					p = xz_p*(xxt*E(l, c, 1)+(1-xxt)*E(l, c, 2));
				}
				else {
					p = (1-xz_p)*(xxt*E(l, c, 1)+(1-xxt)*E(l, c, 2));
				}
				
				// q = P(cell is not traversable) 
			    if (t == 1) {
					q = (1-xz_p)*((1-xxt)*E(l, c, 1)+xxt*E(l, c, 2));
				}
				else {
					q = xz_p*((1-xxt)*E(l, c, 1)+xxt*E(l, c, 2));
				}
				
				E(l,c,1)=p/(p+q);
				E(l,c,2)=q/(p+q);
				
				
				if (E(l,c,1)>E(l,c,2)) {
						E(l,c,0)=1.0;
				}
				else {
						E(l,c,0)=0.0;
				}
				
                mylist.clear();
            }
            
           // Estimation visualization: color cells
           // Green: cell is traversable
           // Blue: cell is non traversable
           // Red: cells is not obeserved
           for(int l=0; l<CN; l++){
			   for(int c=0; c<CN; c++){ 
                	if (E(l, c, 0) == 1.0) {
                    	R(l,c)[0] = 0;
                    	R(l,c)[1] = 255;
                    	R(l,c)[2] = 0;
                	}
                	else if (E(l,c,0) == 0.0){
                   		R(l,c)[0] = 0;
                   		R(l,c)[1] = 0;
                        R(l,c)[2] = 255;
                	}
                	else {
                    	R(l,c)[0] = 255;
                        R(l,c)[1] = 0;
                        R(l,c)[2] = 0;
					}
			   	}
			}
	
            sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", R).toImageMsg();
            image_pub.publish(image_msg);
		}


    public:
            FloorPlaneMapping() : nh_("~") {
                nh_.param("base_frame",base_frame_,std::string("/body"));
                nh_.param("world_frame",world_frame_, std::string("/world"));
                nh_.param("max_range",max_range_,100.0);
                nh_.param("CN",CN,15);
                dCell = 10.0/double(CN);
                
                nh_.param("angle",angle,0.5);
                
                nh_.param("xz",xz,0.8);
                nh_.param("xxt",xxt,0.8);
                
                nh_.param("ransac_n_iteration",ransac_n_iteration,50);
                nh_.param("ransac_tolerance",ransac_tolerance,1.0);
                
                nh_.param("nb_min",nb_min,1000);
                
                ros::Duration(1).sleep();

                ros::NodeHandle(nh2);
                image_transport::ImageTransport image_handler(nh2);
                image_pub = image_handler.advertise("/floor_mapping_b/imageTopic",1);

                //initialize publishers and suscribers
                scan_sub = nh_.subscribe("scans",1,&FloorPlaneMapping::pc_callback,this);

                //initialize iterator
                it=PS.begin();

                //plane 0 = {1,0} traversable or non
                //plane 1 : proba it is traversable at t-1 
                //plane 2 : proba it is not traversable at t-1 
                int sz[3] = {CN,CN,3};

                E = cv::Mat_<double>(3,sz); E=0.5;
                R = cv::Mat_<cv::Vec3b>(CN,CN,cv::Vec3b(0,0,0));
                cv::namedWindow("OPENCV_WINDOW");
            }
};


int main(int argc, char* argv[]) {
    ros::init(argc,argv,"floor_plane_mapping_b");

    FloorPlaneMapping fp;

    ros::spin();


}





