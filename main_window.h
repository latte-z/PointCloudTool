//
// Created by Ryan Shen on 2018/11/6.
//

#ifndef PCL_TRIANGULATION_PCL_VISUALIZER_H
#define PCL_TRIANGULATION_PCL_VISUALIZER_H

// VTK build from source with OpenGL2 Rendering
// These code are not necessary
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
//VTK_MODULE_INIT(vtkInteractionStyle);

#include <QtWidgets/QMainWindow>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/exceptions.h>

#include "pojo/MyPointCloud.h"
#include "ui_main_window.h"


class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

private:
    Ui::MainWindow ui;
    // 存储点云数据
    MyPointCloud my_point_cloud;
    std::vector<MyPointCloud> cloud_vector;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // 存储mesh数据
    pcl::PolygonMesh mesh;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_mesh;

    // 初始化vtk
    void Initial();

    void InitialVtkWidget();

    void InitialMeshVtkWidget();

    // 点云区域框选
    static void pp_callback(const pcl::visualization::AreaPickingEvent &event, void *args);

private slots:

    // File
    void OpenFile();

    void SaveFile();

    void Clear();

    void Exit();

    // View point
    void MainView();

    void LeftView();

    void TopView();

    // View
    void OpenPointsViewer();

    // Algorithm Function
    void DisplayTriangles(int type);

    void DisplayProjection();

    void GetResample();

    void FilterStatisticalOutlierRemoval();


    // remove points
    void RemoveSelectPoints();

    // Console print
    void SaveLog(QString operation, QString note);

};


#endif //PCL_TRIANGULATION_PCL_VISUALIZER_H
