//given an image, compute all models w/rt box and return in a "Shipment" object
bool BoxInspector::model_poses_wrt_box(osrf_gear::Shipment &shipment_status) { // SECOND FUNCTION TO WRITE
    ROS_INFO("model_poses_wrt_box()");
    geometry_msgs::Pose cam_pose, box_pose_wrt_cam, model_pose_wrt_cam, part_pose_wrt_box;
    geometry_msgs::PoseStamped box_pose_wrt_world, part_pose_wrt_world;
    Eigen::Affine3d affine_cam_wrt_world, affine_part_wrt_cam, affine_part_wrt_box,
            affine_box_pose_wrt_world, affine_part_wrt_world, affine_box_pose_wrt_cam;

    get_new_snapshot_from_box_cam();
    bool found_box = false;
    int i_box = 0;
    int num_models;

    num_models = box_inspector_image_.models.size();
    //parse the  image and compute model poses w/rt box; ignore  the box itself
    if (num_models == 0) {
        ROS_WARN("model_poses_wrt_box(): image has zero models");
        return false;
    }
    ROS_INFO("box cam sees %d models", num_models);

    //look for the box:
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;

    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose_wrt_cam = model.pose;
            ROS_INFO_STREAM("box pose w/rt camera: " << box_pose_wrt_cam << endl);
            found_box = true;
            i_box = imodel; //remember where box is in the list of models
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose_wrt_cam);
            affine_box_pose_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_world);
            break;
        }
    }
    if (!found_box) {
        ROS_WARN("model_poses_wrt_box(): did not find box in view");
        return false;
    }

    osrf_gear::Product product;
    //if here, image contains box, and box pose w/rt world is in affine_box_pose_wrt_world
    for (int imodel = 0; imodel < num_models; imodel++) {
        if (imodel != i_box) { //if here, have a model NOT the box
            model = box_inspector_image_.models[imodel];
            string model_name(model.type);
            ROS_INFO_STREAM("model: " << model << endl);
            model_pose_wrt_cam = model.pose;
            part_pose_wrt_world = compute_stPose(cam_pose, model_pose_wrt_cam);
            
            affine_box_pose_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_cam);
            affine_part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(model_pose_wrt_cam);
            affine_part_wrt_box = affine_box_pose_wrt_cam.inverse()*affine_part_wrt_cam;
            part_pose_wrt_box = xformUtils_.transformEigenAffine3dToPose(affine_part_wrt_box);


            product.type = model.type;
            product.pose = part_pose_wrt_box;
            shipment_status.products.push_back(product);
        }
    }
    ROS_INFO_STREAM("resulting part poses w/rt box: " << shipment_status << endl);

}



//given a shipment description that specifies desired parts and  poses with respect to box,
//convert this to poses of parts w/rt world;
//robot needs to know current and desired part poses  w/rt world

void BoxInspector::compute_shipment_poses_wrt_world(osrf_gear::Shipment shipment_wrt_box, geometry_msgs::PoseStamped box_pose_wrt_world,
 vector<osrf_gear::Model> &desired_models_wrt_world) {

    //have the box pose wrt world
    //have shipment specification
    //compute in world coordinates where requested parts belong

    int num_products = shipment_wrt_box.products.size(); //how many models did the camera see?

    if (num_products == 0) return false;
    
    osrf_gear::Model model;
    osrf_gear::Product product;
    geometry_msgs::Pose model_pose;

    for (int i = 0; i < num_models; i++) {
        model = shipment_wrt_box.products[i];
        string model_name(model.type);
        
        model_pose = model.pose;
    
        model.pose = compute_shipment_world_pose(box_pose_wrt_world, model_pose);
        model.type = product.type;
        desired_models_wrt_world.push_back(model);
    }    
}


geometry_msgs::PoseStamped BoxInspector::compute_shipment_world_pose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

    geometry_msgs::PoseStamped stPose_product_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
    //Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    Eigen::Affine3d box_wrt_world, product_wrt_box, product_wrt_world;
    
    box_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
    product_wrt_box = xformUtils_.transformPoseToEigenAffine3d(part_pose);
    product_wrt_world = box_wrt_world*product_wrt_box;
    geometry_msgs::Pose pose_product_wrt_world = xformUtils_.transformEigenAffine3dToPose(product_wrt_world);
    geometry_msgs::PoseStamped product_pose_stamped;
    product_pose_stamped.header.stamp = ros::Time::now();
    product_pose_stamped.header.frame_id = "world";
    product_pose_stamped.pose = pose_product_wrt_world;
    return product_pose_stamped;
}