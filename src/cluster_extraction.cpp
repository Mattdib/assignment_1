#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"
#include <boost/filesystem.hpp>
#include <thread>
#include <iomanip>
#include <sstream>
namespace fs = boost::filesystem;
//#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max) //verifico che il cluster corrente non abbia superato la dimensione massima consentita (iperparametro maxClusterSize)
    {
        cluster.push_back(target_ndx); //aggiungo l'indice del punto corrente al cluster
        visited.insert(target_ndx); //aggiungo l'indice del punto corrente all'insieme dei punti già visitati

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z}; //estraggo le coordinate del punto corrente dalla point cloud
    
        std::vector<int> neighborNdxs = tree->search(point, distanceTol); //eseguo la search nel kdtree per trovare tutti i punti vicini al punto corrente entro la distanza di tolleranza specificata (iperparametro clusterTolerance)

        //per ogni punto vicino trovato, verifico se è già stato visitato e, in caso contrario, richiamo ricorsivamente la funzione proximity per esplorare i suoi vicini
        for (int neighborNdx : neighborNdxs) 
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end()) 
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }
            //Se durante il ciclo la dimensione del cluster raggiunge max, interrompe immediatamente l'esplorazione e risale lo stack delle chiamate (return).
            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
	my_visited_set_t visited{}; //already visited points
	std::vector<pcl::PointIndices> clusters; //vector of PointIndices that will contain all the clusters

    // Scorro tutti i punti della point cloud
    for (int i = 0; i < cloud->size(); ++i)
    {
        // Se il punto non è ancora stato visitato
        if (visited.find(i) == visited.end())
        {
            // Vettore che conterrà gli indici del cluster corrente
            std::vector<int> cluster;

            // Chiamata alla tua funzione proximity per trovare tutti i punti vicini al punto corrente e costruire il cluster (aggiornando visited e cluster)
            proximity(cloud, i, tree, distanceTol, visited, cluster, setMaxClusterSize);

            // Verifico che il cluster rispetti le dimensioni minime e massime richieste
            if (cluster.size() >= setMinClusterSize && cluster.size() <= setMaxClusterSize)
            {
                pcl::PointIndices clusterIndices;
                clusterIndices.indices = cluster; // Salvo gli indici del cluster corrente in clusterIndices
                clusters.push_back(clusterIndices); //aggiungo il cluster corrente alla lista dei cluster trovati
            }
        }
    }

	return clusters;	
}

void 
ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

    // Definisco le PointCloud per i vari step
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // TODO: 1) Downsample the dataset (Voxel Filtering)
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.15f, 0.15f, 0.15f); // Voxel da 15cm x 15cm x 15cm
    vg.filter(*cloud_filtered);

    // TODO: 2) Crop points outside ROI (Region of Interest)
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f(-20.0f, -6.0f, -2.0f, 1.0f));
    cb.setMax(Eigen::Vector4f( 30.0f,  7.0f,  5.0f, 1.0f));
    cb.filter(*cloud_filtered);

    // TODO: 3) Segmentation and apply RANSAC
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);       // 100 iterazioni sono sufficienti poichè ho bisogno di segmentare solo il piano stradale 
    seg.setDistanceThreshold(0.15f); // 15cm di tolleranza dal piano stradale

    // TODO: 4) Segment and remove the planar inliers (Road surface)
    // Per un veicolo il piano stradale è il componente dominante principale: un singolo step risulta la soluzione ottimale.
    seg.setInputCloud(cloud_filtered);
    //funzione che esegue il RANSAC (la segmentation) e salva gli inliers e i coefficienti del piano stimato
    seg.segment(*inliers, *coefficients);

    //Se il numero di inliers è 0, significa che non sono riuscito a stimare un modello planare per il dataset dato (non ci sono abbastanza punti per stimare un piano)
    if (inliers->indices.empty())
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    else
    {

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);

        extract.setNegative(false); //estraggo i punti inliers (cioè quelli appartenenti al piano)
        extract.filter(*cloud_plane); //salvo i punti inliers in cloud_plane

        // Rimozione del piano stradale (restano solo gli ostacoli/oggetti sopra il piano stradale)
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
    }

    // 1. Il piano stradale stimato (Verde) (WORKING)
    //renderer.RenderPointCloud(cloud_plane, "roadPlane", Color(0, 1, 0));

    // Visualizza tutti i punti filtrati e ritagliati (in bianco/grigio chiaro) (WORKING)
    //renderer.RenderPointCloud(cloud_filtered, "filteredCloud", Color(1, 1, 1));

    // TODO: 5) Create the KDTree and the vector of PointIndices
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices; // Vettore che conterrà gli indici dei cluster

    // Parametri calibrati per LiDAR su scala teorica "reale"
    float clusterTolerance = 0.35f;  // 35cm: collega i punti dello stesso veicolo senza fondere auto affiancate
    int setMinClusterSize = 15;      // Permette di rilevare pedoni e auto lontane
    int setMaxClusterSize = 1000;    // Taglia pareti o guardrail continui

    #ifdef USE_PCL_LIBRARY
        // TODO: 6) PCL Euclidean Clustering
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //creo l'oggetto EuclideanClusterExtraction utile per eseguire l'estrazione dei cluster
        ec.setClusterTolerance (clusterTolerance); // 35cm (If you take a very small value, it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster)
        //We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points
        ec.setMinClusterSize (setMinClusterSize);
        ec.setMaxClusterSize (setMaxClusterSize);
        ec.setSearchMethod (tree); //setto come metodo di ricerca il kdtree che ho creato prima
        ec.setInputCloud (cloud_filtered); //setto come input cloud il cloud filtrato (cioè senza il piano)
        ec.extract (cluster_indices); // la funzione extract() esegue l'estrazione dei cluster e salva gli indici dei punti di ciascun cluster in cluster_indices
    #else
        // Implementazione custom KD-Tree
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3);
        cluster_indices = euclideanCluster(cloud_filtered, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);
    #endif

/*

    Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry (cluster) and write all points of the current cluster in the PointCloud.
    Compute euclidean distance

*/

    // Nota: il piano stradale (es. cloud_plane) conviene renderizzarlo UNA SOLA VOLTA prima del ciclo:
    renderer.RenderPointCloud(cloud_plane, "roadPlane", Color(0, 1, 0)); // Verde per visualizzare i punti del piano stradale (cloud_plane)

    //int j = 0;
    int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) // scorro tutti i cluster trovati
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>); //pointcloud che conterrà i punti del cluster corrente
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) //scorro tutti i punti del cluster corrente
            cloud_cluster->push_back ((*cloud_filtered)[*pit]); //salvo i punti del cluster corrente in cloud_cluster
        cloud_cluster->width = cloud_cluster->size (); //setto la larghezza del cloud_cluster al numero di punti del cluster corrente
        cloud_cluster->height = 1; //setto l'altezza del cloud_cluster a 1 (perché è un cloud 2D)
        cloud_cluster->is_dense = true; //setto is_dense a true (perché non ci sono punti NaN nel cloud_cluster)

        // TODO: 7) render the cluster and plane without rendering the original cloud 
        renderer.RenderPointCloud(cloud_cluster, "cluster" + std::to_string(clusterId), Color(1,1,1)); //Binco per tutti i cluster, poi coloro solo quelli critici in rosso

        // Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt); 

        Box box{minPt.x, minPt.y, minPt.z,
                maxPt.x, maxPt.y, maxPt.z};

        // TODO: 8) Plot the distance of each cluster w.r.t ego vehicle
        float centerX = (minPt.x + maxPt.x) / 2.0f;
        float centerY = (minPt.y + maxPt.y) / 2.0f;
        float distance = std::hypot(minPt.x, centerY); // calcolo la distanza euclidea tra il sensore e la parte posteriore del veicolo/ostacolo (std::hypot(x, y) equivale a sqrt(x^2 + y^2))

        std::cout << "Cluster " << clusterId << " - Distanza: " << distance << " m" << std::endl;
        // Formatta la distanza a una cifra decimale
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << distance << " m";
        std::string distStr = ss.str();

        // Posiziona il testo appena sopra il tetto del box (maxPt.z + 0.4 metri)
        std::string textId = "dist_label_" + std::to_string(clusterId);
        renderer.addText(centerX, centerY, maxPt.z + 0.4f, distStr, textId);

        // TODO: 9) Color the vehicles that are both in front and 5 meters away from the ego vehicle (probabilmente sarebbe più opportuno considerare una distanza più elevata, ad esempio 10 metri, soprattutto se il veicolo che segue si trova dinanzi a noi)

        // Verifichiamo che sia davanti (minPt.x > 0) ed entro 5 metri (verifico anche che si trovi nella mia stessa corsia di marcia, che supponiamo sia di 3 metri di larghezza).
        // Parametri geometrici della corsia di marcia (in metri)
        const float lane_half_width = 1.5f; // Corsia di marcia teorica larga 3 metri (-1.5m a +1.5m)
        const float max_critical_distance = 5.0f; // Soglia di attenzione critica (Rosso)
        const float max_warning_distance = 10.0f;  // Soglia di attenzione intermedia (Giallo)


        // 1. Verifica laterale: l'ostacolo invade la nostra corsia?
        bool inMyLane = (minPt.y <= lane_half_width && maxPt.y >= -lane_half_width);

       // 2. Classificazione della distanza longitudinale lungo la nostra traiettoria
        bool isCritical = inMyLane && (minPt.x > 0.0f && minPt.x <= max_critical_distance);
        bool isWarning  = inMyLane && (minPt.x > max_critical_distance && minPt.x <= max_warning_distance);

        if (isCritical) 
        {
            // Veicolo nella nostra traiettoria entro 5m -> Box ROSSO
            renderer.RenderBox(box, clusterId, Color(1, 0, 0)); 
        }
        else if (isWarning)
        {
            // Veicolo nella nostra traiettoria tra 5m e 10m -> Box GIALLO
            renderer.RenderBox(box, clusterId, Color(1, 1, 0)); 
        }
        else 
        {
            // Veicoli oltre i 10m, nelle altre corsie o dietro -> Box CIANO
            renderer.RenderBox(box, clusterId, Color(0, 1, 1)); 
        }

        ++clusterId;
    }
    
}


int main(int argc, char* argv[])
{
    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/home/ilpiova/Autonomous/assignment/assignment_1/dataset_2"},
                                                boost::filesystem::directory_iterator{});

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read (streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        /*std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
        << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;*/

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(40));

    }
}
