//#include "render/render.h"

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
        :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

struct KdTree
{
    Node* root;
    int dimension{2};

    KdTree()
        : root(NULL)
    {}
    void insertHelper(Node *&rootNode, std::vector<float> pt, int id, int depth)
    {
        int curr_depth = depth%dimension;

        if (rootNode == nullptr)
        {
            rootNode = new Node(pt, id);
        }
        else if(pt[curr_depth] < rootNode->point[curr_depth])
        {
            insertHelper(rootNode->left, pt, id, depth+1);
        }
        else
        {
            insertHelper(rootNode->right, pt, id, depth+1);
        }
    }

    void insert(std::vector<float> point, int id)
    {
        if(root == nullptr)
        {
            root = new Node(point, id);
        }
        else
        {
            insertHelper(root, point, id, 0);
        }
    }

    void searchHelper(std::vector<int> &id, std::vector<float> target, float distThersh, Node *rootNode, int depth)
    {
        //The following piece of code is written entierly by BROLIN.
        int curr_depth = depth%dimension;

        if (rootNode == nullptr)
            return;

        bool threshold_passed{true};

        //check if distance threshold is passed in all dimensions
        for(int i=0; i<dimension; i++)
        {
            bool passed;
            passed = (std::fabs(target[i] - rootNode->point[i]) < distThersh);
            threshold_passed = threshold_passed && passed;
        }

        //if it passes the threshold, then see if eucledian distance passes the threshold
        //and push it to id
        if(threshold_passed)
        {
            float dist_ = 0;
            for (int i=0; i<dimension; i++)
                dist_ += (std::pow((target[i] - rootNode->point[i]), 2));

            if(dist_ < distThersh)
                id.push_back(rootNode->id);
        }

        if(target[curr_depth] <= rootNode->point[curr_depth])
        {
            searchHelper(id, target, distThersh, rootNode->left, depth+1);

            if(std::fabs(target[curr_depth] - rootNode->point[curr_depth]) <= distThersh)
                searchHelper(id, target, distThersh, rootNode->right, depth+1);
        }
        else
        {
            searchHelper(id, target, distThersh, rootNode->right, depth+1);

            if(std::fabs(target[curr_depth] - rootNode->point[curr_depth]) <= distThersh)
                searchHelper(id, target, distThersh, rootNode->left, depth+1);
        }
      /*
        // The following lines of code are standard from Udacity course.
        int curr_depth = depth%2;

        if (rootNode == nullptr)
        {
            return;
        }

        float x_diff = (target[0] - rootNode->point[0]);
        float y_diff = (target[1] - rootNode->point[1]);


        if(rootNode->point[0] >= (target[0] - distThersh) && rootNode->point[0] <= (target[0] + distThersh) &&
           rootNode->point[1] >= (target[1] - distThersh) && rootNode->point[1] <= (target[1] + distThersh) )
        {
            if(std::sqrt(x_diff* x_diff + y_diff*y_diff) <= distThersh)
            id.push_back(rootNode->id);
        }

        if((target[curr_depth] - distThersh)< rootNode->point[curr_depth])
        {
            searchHelper(id, target, distThersh, rootNode->left, depth+1);
        }
        if((target[curr_depth] + distThersh) > rootNode->point[curr_depth])
        {
            searchHelper(id, target, distThersh, rootNode->right, depth+1);
        }*/

    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(ids, target, distanceTol, root, 0);
        return ids;
    }


};




