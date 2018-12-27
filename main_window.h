//
// Created by Ryan Shen on 2018/11/6.
//

#ifndef PCL_TRIANGULATION_PCL_VISUALIZER_H
#define PCL_TRIANGULATION_PCL_VISUALIZER_H

#ifdef Q_OS_MAC
#include <Carbon/Carbon.h>
#include <ApplicationServices/ApplicationServices.h>
#endif

// Visualization Toolkit
// VTK build from source with OpenGL2 Rendering
// These code are not necessary on macOS
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
//VTK_MODULE_INIT(vtkInteractionStyle);
#include <vtkRenderWindow.h>

#include <iostream>
#include <vector>
#include <QFileDialog>
#include <QtWidgets/QWidget>
#include <QMessageBox>
#include <QSignalMapper>
#include <QDateTime>
#include <QtWidgets/QMainWindow>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/exceptions.h>
#include <pcl/visualization/cloud_viewer.h>

#include "tool/triangulation.h"
#include "tool/classifier.h"
#include "tool/projection.h"
#include "tool/resampling.h"
#include "tool/filter.h"
#include "tool/utils.h"
#include "tool/io.h"
#include "pojo/MyException.h"
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
    MyPointCloud myPointCloud;
    std::vector<MyPointCloud> cloud_vector;
    long points_number = 0;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // 存储mesh数据
    pcl::PolygonMesh mesh;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_mesh;

    // 初始化
    void Initial();

    // 点云区域框选
    static void pp_callback(const pcl::visualization::AreaPickingEvent &event, void *args);

    // 更新viewer
    void updateViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

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


    void toggleSelectPoints(bool isToggle);
    // remove points
    void RemoveSelectPoints();

    // Region Growing
    void RegionGrowing();

    // Console print
    void SaveLog(QString operation, QString note);

};


#endif //PCL_TRIANGULATION_PCL_VISUALIZER_H
