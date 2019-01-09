
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>

using namespace std;

typedef struct txtPoint_3D {
    double x;
    double y;
    double z;
    int r;
    int g;
    int b;
} txtPoint_3D;

int main() {
    FILE *fp_txt;
    txtPoint_3D txtPoint;
    vector<txtPoint_3D> txtPoints;
    fp_txt = fopen("/Users/Ryan/Developer/point_cloud_tool/data/railway-colors.txt", "r");
    // 将txt文本中的点信息存入vector中
    if (fp_txt) {
        while (fscanf(fp_txt, "%lf %lf %lf %d %d %d", &txtPoint.x, &txtPoint.y, &txtPoint.z, &txtPoint.r,
                      &txtPoint.g, &txtPoint.b) != EOF) {
            txtPoints.push_back(txtPoint);
        }
    }
    fclose(fp_txt);
    // 新建一个点云文件
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.width = txtPoints.size();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    int frgb = 0;
    // 传递点云信息
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = txtPoints[i].x;
        cloud.points[i].y = txtPoints[i].y;
        cloud.points[i].z = txtPoints[i].z;
        // rgb 处理特殊，不可以直接赋值
        frgb = ((txtPoints[i].r) << 16 | (txtPoints[i].g) << 8 | (txtPoints[i].b));
        cloud.points[i].rgb = *reinterpret_cast<float*>(&frgb); // 此处由int型转成了float型
    }
    pcl::io::savePCDFileASCII("railway-colors.pcd", cloud);
    return 0;
}