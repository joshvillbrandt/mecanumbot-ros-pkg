#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <std_msgs/String.h>
#include <sstream>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <LinearMath/btVector3.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#define stddev(x, u, s) (x > (u-s) && x < (u+s))


template<typename T>
inline bool isnan(T value)
{
	return value != value;
}

class Color
{
  public:
	std::string name;
	uint8_t r_u, r_s;
	uint8_t g_u, g_s;
	uint8_t b_u, b_s;

	Color () :
	  r_u (0),
	  r_s (0),
	  g_u (0),
	  g_s (0),
	  b_u (0),
	  b_s (0)
	{}

    ~Color () {}

    /*void
    ROSInfo()
    {
		std::stringstream ss;
		ss << "(r_u, r_s, g_u, g_s, b_u, b_s): " << name << " ";
		ss << (int)r_u << " " << (int)r_s << " " << (int)g_u;
		ss << " " << (int)g_s << " " << (int)b_u << " " << (int)b_s;
		ROS_INFO("%s", ss.str().c_str());
    }*/
};

class FeatureCloud
{
  public:
	std::string name;
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::KdTree<pcl::PointXYZRGB> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new pcl::KdTreeFLANN<pcl::PointXYZRGB>),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
    	search_method_xyz_->setInputCloud(xyz_);
		computeSurfaceNormals ();
		computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
		normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
		norm_est.setInputCloud (xyz_);
		norm_est.setSearchMethod (search_method_xyz_);
		norm_est.setRadiusSearch (normal_radius_);
		norm_est.compute (*normals_);
	}

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
		features_ = LocalFeatures::Ptr (new LocalFeatures);

		pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
		fpfh_est.setInputCloud (xyz_);
		fpfh_est.setInputNormals (normals_);
		fpfh_est.setSearchMethod (search_method_xyz_);
		fpfh_est.setRadiusSearch (feature_radius_);
		fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:
    std::vector<FeatureCloud> templates_;

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f), // started as 0.05f
      max_correspondence_distance_ (0.01f*0.01f), // started as 0.01f*0.01f
      nr_iterations_ (500)
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZRGB> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

class CloudHelpers
{
  public:
	TemplateAlignment template_align;
	std::vector<Color> colors;

	CloudHelpers ()
	{
	}

	~CloudHelpers () {}

	int
	identifyShape(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		// set target cloud
		FeatureCloud target_cloud;
		target_cloud.setInputCloud(cloud);
		template_align.setTargetCloud(target_cloud);

		// find the best template alignment
		TemplateAlignment::Result best_alignment;
		int best_index = template_align.findBestAlignment (best_alignment);
		//const FeatureCloud &best_template = object_templates[best_index];
		//ROS_INFO("Fitness score: %f", best_alignment.fitness_score);

		return best_index;
	}

template<typename T>
inline bool isnan(T value)
{
return value != value;
}
	int
	identifyColor(uint8_t r, uint8_t g, uint8_t b)
	{
		int bestIndex = -1, lowestError = 1000, error;
		float lowLight = 1;//0.60f; // for night time in my bed room
		float devAmt = 2; // number of std devs to accept

		for(size_t i = 0; i < colors.size(); i++)
		{
			// check to see if we are in range
			if(stddev(r, colors[i].r_u*lowLight, colors[i].r_s*lowLight*devAmt) &&
				stddev(g, colors[i].g_u*lowLight, colors[i].g_s*lowLight*devAmt) &&
				stddev(b, colors[i].b_u*lowLight, colors[i].b_s*lowLight*devAmt))
			{
				error = abs(colors[i].r_u*lowLight - r) + abs(colors[i].g_u*lowLight - g) + abs(colors[i].b_u*lowLight - b);
				if(error < lowestError)
				{
					lowestError = error;
					bestIndex = i;
				}
			}
		}

		return bestIndex;
	}

	pcl::ModelCoefficients::Ptr
	identifyOrientation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		// Separate largest contiguous plane (floor) via RANSAC
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (1000);
		seg.setDistanceThreshold (0.001);
		seg.setOptimizeCoefficients (true); // optional
		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);

		// Debug surface
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud);

		return coefficients;
	}

	void
	loadConfig(char *config_filename)
	{
		std::ifstream input_stream (config_filename);
		std::string line;
		enum readStates {SHAPES, COLORS, GENERAL};
		readStates read_state = GENERAL;
		while (input_stream.good ())
		{
			std::getline (input_stream, line);
			if (line.empty () || line.at (0) == '#') // Skip blank lines or comments
				continue;
			else if (strcmp(line.c_str(), "SHAPES") == 0) {
				read_state = SHAPES;
				continue;
			}
			else if (strcmp(line.c_str(), "COLORS") == 0) {
				read_state = COLORS;
				continue;
			}

			// find first space in line
			size_t pos = line.find(" ");

			if(read_state == SHAPES)
			{
				// create and add FeatureCloud
				FeatureCloud template_cloud;
				template_cloud.name = line.substr(0, pos);
				template_cloud.loadInputCloud (line.substr(pos+1));
				template_align.addTemplateCloud (template_cloud);

				// let the world know
				std::stringstream ss;
				ss << "Shape loaded: " << template_cloud.name << " with ";
				ss << template_cloud.getPointCloud()->size() << " points";
				ROS_INFO("%s", ss.str().c_str());
			}
			else if(read_state == COLORS)
			{
				// create and add Color
				Color color;
				color.name = line.substr(0, pos);

				int r_u, r_s, g_u, g_s, b_u, b_s;
				sscanf (line.substr(pos+1).c_str(),"%d %d %d %d %d %d", &r_u, &r_s, &g_u, &g_s, &b_u, &b_s);

				color.r_u = (uint8_t)r_u;
				color.r_s = (uint8_t)r_s;
				color.g_u = (uint8_t)g_u;
				color.g_s = (uint8_t)g_s;
				color.b_u = (uint8_t)b_u;
				color.b_s = (uint8_t)b_s;

				colors.push_back(color);

				// let the world know
				std::stringstream ss;
				ss << "Color loaded: " << color.name << " (R";
				ss << (int)r_u << " " << (int)r_s << " G" << (int)g_u;
				ss << " " << (int)g_s << " B" << (int)b_u << " " << (int)b_s << ")";
				ROS_INFO("%s", ss.str().c_str());
			}
		}
		input_stream.close ();
	}

	void
	vectorAppend(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &tothis)
	{
		for(size_t i = 0; i < in.size(); i++)
		{
			tothis.push_back(in[i]);
		}
	}

	void
	extractRGB(pcl::PointXYZRGB point, uint8_t &r, uint8_t &g, uint8_t &b)
	{
		uint32_t rgb_val_;
		memcpy(&rgb_val_, &(point.rgb), sizeof(float));
		r = (uint8_t)((rgb_val_ >> 16) & 0x000000ff);
		g = (uint8_t)((rgb_val_ >> 8) & 0x000000ff);
		b = (uint8_t)((rgb_val_) & 0x000000ff);
	}

	pcl::PointXYZRGB
	averageCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, uint8_t &r_ret, uint8_t &g_ret, uint8_t &b_ret)
	{
		pcl::PointXYZRGB avg;
		avg.x = 0; avg.y = 0; avg.z = 0;
		long r_avg = 0, g_avg = 0, b_avg = 0;

		for(size_t i = 0; i < cloud->points.size(); i++)
		{
			if(!isnan(cloud->points[i].x) && !isnan(cloud->points[i].y) && !isnan(cloud->points[i].z)) {
				avg.x += cloud->points[i].x;
				avg.y += cloud->points[i].y;
				avg.z += cloud->points[i].z;

				uint8_t r, g, b;
				extractRGB(cloud->points[i], r, g, b);
				r_avg += r;
				g_avg += g;
				b_avg += b;
			}
		}
		
		//std::cerr << "Average of " << cloud->points.size() << " points: "
		//	<< avg.x << " " << avg.x << " " << avg.x << std::endl;

		avg.x /= cloud->points.size();
		avg.y /= cloud->points.size();
		avg.z /= cloud->points.size();

		r_ret = r_avg / cloud->points.size();
		g_ret = g_avg / cloud->points.size();
		b_ret = b_avg / cloud->points.size();

		return avg;
	}

	void
	colorStats(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		uint8_t r_avg = 0, g_avg = 0, b_avg = 0;
		averageCloud(cloud, r_avg, g_avg, b_avg);
		long r_sigma = 0, g_sigma = 0, b_sigma = 0; // one standard deviation

		for(size_t i = 0; i < cloud->points.size(); i++)
		{
			uint8_t r, g, b;
			extractRGB(cloud->points[i], r, g, b);
			r_sigma += sqrt((r-r_avg)*(r-r_avg));
			g_sigma += sqrt((g-g_avg)*(g-g_avg));
			b_sigma += sqrt((b-b_avg)*(b-b_avg));
		}

		r_sigma /= cloud->points.size();
		g_sigma /= cloud->points.size();
		b_sigma /= cloud->points.size();

		std::stringstream ss;
		ss << "(r_u, r_s, g_u, g_s, b_u, b_s): ";
		ss << (int)r_avg << " " << r_sigma << " " << (int)g_avg;
		ss << " " << g_sigma << " " << (int)b_avg << " " << b_sigma;
		ROS_INFO("%s", ss.str().c_str());
	}

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
	segmentByColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		// Create index groups for each color
		std::vector<pcl::PointIndices::Ptr> indicies;
		for(size_t i = 0; i < colors.size(); i++)
		{
			pcl::PointIndices::Ptr indexGroup (new pcl::PointIndices ());
			indicies.push_back(indexGroup);
		}

		// Segment by color
		for(size_t i = 0; i < cloud->points.size(); i++)
		{
			uint8_t r, g, b;
			extractRGB(cloud->points[i], r, g, b);
			int bestIndex = identifyColor(r, g, b);

			if(bestIndex != -1) indicies[bestIndex]->indices.push_back(i);
		}

		// Create clouds from indices
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
		for(size_t i = 0; i < colors.size(); i++)
		{
			// Create cloud if big enough
			size_t minSize = 50;
			//std::cout << "indices: " << indicies[i]->indices.size() << std::endl;
			if(indicies[i]->indices.size() > minSize)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::ExtractIndices<pcl::PointXYZRGB> extract;
				extract.setInputCloud (cloud);
				extract.setIndices (indicies[i]);
				extract.setNegative (false);
				extract.filter (*cluster);

				cluster->width = cluster->points.size ();
				cluster->height = 1;
				cluster->is_dense = true;

				clusters.push_back(cluster);
			}
		}

		return clusters;
	}

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
	segmentByDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		// Creating the KdTree object and perform Euclidean distance search
		pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
		tree->setInputCloud (cloud);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance (0.01); // 1 cm
		ec.setMinClusterSize (50);
		ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud(cloud);
		ec.extract (cluster_indices);

		// Separate clusters
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				cloud_cluster->points.push_back (cloud->points[*pit]); //
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			cloud_cluster->header = cloud->header;

			clusters.push_back(cloud_cluster);
		}

		return clusters;
	}

	// find points behind a plane (opposite of the normal vector given in ax+by+cz+d=0)
	pcl::PointIndices::Ptr
	filterBehindPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients)
	{
		pcl::PointIndices::Ptr indicies (new pcl::PointIndices ());
		float d = 0.0f; // part of the distance formula from eq 10 at http://mathworld.wolfram.com/Point-PlaneDistance.html
		for(size_t i = 0; i < cloud->points.size(); i++)
		{
			d = coefficients->values[0] * cloud->points[i].x + // a * x0
					coefficients->values[1] * cloud->points[i].y + // b * y0
					coefficients->values[2] * cloud->points[i].z + // c * z0
					coefficients->values[3]; // d
			if(d < 0)
			{
				indicies->indices.push_back(i);
			}
		}

		return indicies;
	}

	pcl::PointIndices::Ptr
	filterColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Color color)
	{
		pcl::PointIndices::Ptr indicies (new pcl::PointIndices ());
		uint8_t r, g, b;
		for(size_t i = 0; i < cloud->points.size(); i++)
		{
			extractRGB(cloud->points[i], r, g, b);

			// check to see if we are in range
			if(stddev(r, color.r_u, color.r_s) && stddev(g, color.g_u, color.g_s) && stddev(b, color.b_u, color.b_s))
			{
				indicies->indices.push_back(i);
			}
		}

		return indicies;
	}
};
