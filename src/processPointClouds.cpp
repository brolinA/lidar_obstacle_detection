// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cropped_cloud (new pcl::PointCloud<PointT>);

    // Create the filtering object
     typename pcl::VoxelGrid<PointT> vox_fileter_;
     vox_fileter_.setInputCloud (cloud);
     vox_fileter_.setLeafSize (filterRes, filterRes, filterRes);
     vox_fileter_.filter (*voxel_cloud);

     typename pcl::CropBox<PointT> cropbox_filter_;
     //first crop the interest region
     cropbox_filter_.setMin(minPoint);
     cropbox_filter_.setMax(maxPoint);
     cropbox_filter_.setInputCloud(voxel_cloud);
     cropbox_filter_.filter(*cropped_cloud);

     //filter the roof points
     cropbox_filter_.setMin(Eigen::Vector4f(-3.0, -3.0, -1.0, 1.0));
     cropbox_filter_.setMax(Eigen::Vector4f(3.0, 3.0, 1.0, 1.0));
     cropbox_filter_.setInputCloud(cropped_cloud);
     cropbox_filter_.setNegative(true);
     cropbox_filter_.filter(*cropped_cloud);

    return cropped_cloud;
}

template<typename PointT>
std::vector<float> ProcessPointClouds<PointT>::CrossProduct(std::vector<float> v1, std::vector<float> v2)
{
    std::vector<float> result_;
    if(v1.size() != v2.size())
    {
        std::cout<< "ERROR: The vector size does not match to do cross product"<<std::endl;
        return result_;
    }
    result_.resize(v1.size());
    result_[0] = v1[1]*v2[2] - v1[2]*v2[1];
    result_[1] = v1[2]*v2[0] - v1[0]*v2[2];
    result_[2] = v1[0]*v2[1] - v1[1]*v2[0];

    return result_;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    for(int i=0; i<maxIterations; i++)
    {
        std::unordered_set<int> inlierPts;
        //this will make sure we are not choosing the same point again.
        while (inlierPts.size()<3)
            inlierPts.insert(rand()%cloud->points.size());

        float x1, x2, x3;
        float y1, y2, y3;
        float z1, z2, z3;
        auto it = inlierPts.begin();
        x1 = cloud->points[*it].x;
        y1 = cloud->points[*it].y;
        z1 = cloud->points[*it].z;
        it++;
        x2 = cloud->points[*it].x;
        y2 = cloud->points[*it].y;
        z2 = cloud->points[*it].z;
        it++;
        x3 = cloud->points[*it].x;
        y3 = cloud->points[*it].y;
        z3 = cloud->points[*it].z;

        std::vector<float> v1, v2;

        v1.push_back(x2-x1);
        v1.push_back(y2-y1);
        v1.push_back(z2-z1);

        v2.push_back(x3-x1);
        v2.push_back(y3-y1);
        v2.push_back(z3-z1);

        //find the cross product of v1 x v2
        std::vector<float> cross_p_ = CrossProduct(v1, v2);
        if(cross_p_.size() != 0)
        {
            float a_ = cross_p_[0];
            float b_ = cross_p_[1];
            float c_ = cross_p_[2];
            float d_ = -(a_*x1 + b_*y1 + c_*z1);

            for (int index=0; index<cloud->points.size(); index++)
            {
                if(inlierPts.count(index) > 0)
                    continue;

                PointT point_ = cloud->points[index];
                float x_, y_, z_;
                x_ = point_.x;
                y_ = point_.y;
                z_ = point_.z;

                float d = std::fabs(a_*x_ + b_*y_ + c_*z_ + d_)/std::sqrt(a_*a_ + b_*b_ + c_*c_);

                if(d<=distanceThreshold)
                    inlierPts.insert(index);
            }

            if(inlierPts.size() > inliersResult.size()){
                inliersResult = inlierPts;
            }

        }
        else
        {
            std::cout<<"ERROR: Invalid CrossProduct"<<std::endl;
            break;
        }
    }

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult (cloudOutliers, cloudInliers);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

   /* // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    typename std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr kd_tree (new pcl::search::KdTree<PointT>);
    kd_tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (kd_tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
     {
       typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
         cloud_cluster->push_back ((*cloud)[*pit]);
       cloud_cluster->width = cloud_cluster->size ();
       cloud_cluster->height = 1;
       cloud_cluster->is_dense = true;

       clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
   return clusters;*/

    KdTree* tree = new KdTree;
    tree->dimension = 3;

    for (int i=0; i<cloud->points.size(); i++)
    {
        std::vector<float> pt;
        pt.push_back(cloud->points[i].x);
        pt.push_back(cloud->points[i].y);
        pt.push_back(cloud->points[i].z);
        tree->insert(pt,i);
    }
    std::vector<typename pcl::PointCloud<PointT>::Ptr > final_cluster = euclideanCluster(*cloud, tree, clusterTolerance, minSize, maxSize);

    return final_cluster;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr > ProcessPointClouds<PointT>::euclideanCluster(const typename pcl::PointCloud<PointT> &input_cloud, KdTree* tree, float distanceTol, int min_point, int max_points)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr > clusters;
    std::vector<std::pair<std::vector<float>, int> > unprocessed_nodes;
    unprocessed_nodes.push_back(std::make_pair(tree->root->point, tree->root->id));

    //getting all the nodes present in the tree.
    getAllNodes(tree->root, unprocessed_nodes);

    while(unprocessed_nodes.size() != 0)
    {
        std::vector<int> indices;
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        proximityCheck(unprocessed_nodes[0].first, unprocessed_nodes[0].second, indices, unprocessed_nodes, tree, distanceTol, max_points);

        //update cluster only if it is greater than the defined minimum
        if(indices.size() > min_point)
        {
            for (std::vector<int>::const_iterator pit = indices.begin (); pit != indices.end (); ++pit)
                cloud_cluster->push_back((input_cloud)[*pit]);

            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
        }
    }
    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximityCheck(std::vector<float> check_pt_, int id, std::vector<int> &final_cluster, std::vector<std::pair<std::vector<float>, int> > &unchecked_nodes, KdTree* tree, float tolerance, int max_points)
{
    if(final_cluster.size() < max_points)
    {
        final_cluster.push_back(id);

        removeID(id, unchecked_nodes);

        std::vector<int> ids_ = tree->search(check_pt_, tolerance);

        for(int i = 0; i < ids_.size(); i++ )
        {
            //Check if the point in unchecked and then process the point.
            for(auto it = unchecked_nodes.begin(); it != unchecked_nodes.end(); it++ )
            {
                if (it->second == ids_[i])
                {
                    proximityCheck(it->first, ids_[i], final_cluster, unchecked_nodes, tree, tolerance, max_points);
                    break;
                }
            }
        }
    }
}

template<typename PointT>
void  ProcessPointClouds<PointT>::extractCloudFromIndices(const std::vector<int> &indices, pcl::PointCloud<pcl::PointXYZI> input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &output_cluster)
{
    for(int index: indices)
    {
        output_cluster->points.push_back(input_cloud.points[index]);
    }
}


template<typename PointT>
void ProcessPointClouds<PointT>::removeID(int id, std::vector<std::pair<std::vector<float>, int> > &point_list)
{
    //remove a given id from the list of nodes and their corresponding id
    for(auto it = point_list.begin(); it != point_list.end(); it++ )
    {
        if (it->second == id){
            point_list.erase(it);
            break;
        }
    }
}

template<typename PointT>
void ProcessPointClouds<PointT>::getAllNodes(Node *curr_node, std::vector<std::pair<std::vector<float> , int> > &nodes)
{
        if(curr_node->left != nullptr)
        {
            nodes.push_back(std::make_pair(curr_node->left->point, curr_node->left->id));
            getAllNodes(curr_node->left, nodes);
        }
        if(curr_node->right != nullptr)
        {
            nodes.push_back(std::make_pair(curr_node->right->point, curr_node->right->id));
                getAllNodes(curr_node->right, nodes);
        }
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
