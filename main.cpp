#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr ConstructWhiteDash(int a, int b);

PointCloudT::Ptr ConstructStraightArrow(int a, int b, int c, int d);

PointCloudT::Ptr ConstructTurningArrow(int a, int b, int c, int d, int e, int f, int g, int h, int m);

PointCloudT::Ptr ConstructComplexArrow(int a, int b, int c, int d, int e, int f, int g, int h, int m, int n, int p);


PointCloudT::Ptr ConstructDoubleArrow(int a, int b, int c, int d, int e, int f, int g, int h, int m);

PointCloudT::Ptr ConstructDoubleComplexArrow(int a, int b, int c, int d, int e, int f, int g, int h, int m, int n, int p);


using std::cout;
using std::endl;
using std::cerr;

int main() {
    PointCloudT::Ptr cloud(new PointCloudT);
    // Step1: Construct white dash
    cloud = ConstructWhiteDash(15, 205);
    pcl::io::savePLYFile("../templates/WhiteDash.ply", *cloud);

    // Step2: Construct straight arrow
    cloud = ConstructStraightArrow(16, 183, 14, 120);
    pcl::io::savePLYFile("../templates/StraightArrow.ply", *cloud);

    // Step3: Construct turning arrow
    cloud = ConstructTurningArrow(16, 150, 194, 40, 41, 18, 70, 76, 58) ;
    pcl::io::savePLYFile("../templates/TurningArrow.ply", *cloud);

    // Step5: Construct complex arrow
    cloud = ConstructComplexArrow(15, 23, 180, 40, 48, 18, 74, 75, 53, 15, 120);
    pcl::io::savePLYFile("../templates/ComplexArrow.ply", *cloud);

    cloud = ConstructDoubleComplexArrow(16, 18, 126, 40, 42, 30, 72, 78, 60, 15, 120);
    pcl::io::savePLYFile("../templates/DoubleComplexArrow.ply", *cloud);
}

PointCloudT::Ptr ConstructWhiteDash(int a, int b) {
    std::vector<Eigen::Vector2d> vPts;
    for (int i = 0; i <= a; ++i) {
        for (int j = 0; j <= b; ++j) {
            vPts.emplace_back(i, j);
        }
    }

    // 去中心化
    int numPts = vPts.size();
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto &pt : vPts) {
        centroid += pt;
    }
    centroid /= numPts;

    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = numPts;
    cloud->height = 1;
    cloud->points.resize(numPts);
    for (int i = 0; i < numPts; ++i) {
        Eigen::Vector2d pt = vPts[i] - centroid;
        auto &point = cloud->points[i];
        point.x = 0.01 * pt.x();
        point.y = 0.01 * pt.y();
        point.z = 0;
    }

    return cloud;
}

PointCloudT::Ptr ConstructStraightArrow(int a, int b, int c, int d) {
    std::vector<Eigen::Vector2d> vPts;
    double k = -d / (0.5*a + c);
    for (int x = 0; x <= 0.5*a + c; ++x) {
        double Y = k*x + d;
        for (int y = 0; y < Y; ++y) {
            vPts.emplace_back(x, y);
            vPts.emplace_back(-x, y);
        }
    }
    for (int x = 0; x <= 0.5*a; ++x) {
        for (int y = -b; y < 0; ++y) {
            vPts.emplace_back(x, y);
            vPts.emplace_back(-x, y);
        }
    }


    // 去中心化
    int numPts = vPts.size();
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto &pt : vPts) {
        centroid += pt;
    }
    centroid /= numPts;

    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = numPts;
    cloud->height = 1;
    cloud->points.resize(numPts);
    for (int i = 0; i < numPts; ++i) {
        Eigen::Vector2d pt = vPts[i] - centroid;
        auto &point = cloud->points[i];
        point.x = 0.01 * pt.x();
        point.y = 0.01 * pt.y();
        point.z = 0;
    }

    return cloud;
}

PointCloudT::Ptr ConstructTurningArrow(int a, int b, int c, int d, int e, int f, int g, int h, int m) {
    std::vector<Eigen::Vector2d> vPts;
    for (int i = 0; i <= a; ++i) {
        for (int j = 0; j <= i + (c-m); ++j) {
            vPts.emplace_back(i, j);
        }
    }
    for (int i = 0; i <= a+d; ++i) {
        for (int j = i + (c-m); j < i + c; ++j) {
            vPts.emplace_back(i, j);
        }
    }

    std::vector<Eigen::Vector2d> vPts_triangle;
    double k1 = -double(f) / g;
    double k2 = double(f) / h;
    for (int i = 0; i <= g; ++i) {
        for (int j = 0; j < k1 * i + f; ++j) {
            vPts_triangle.emplace_back(i, j);
        }
    }
    for (int i = -h; i < 0; ++i) {
        for (int j = 0; j < k2*i + f; ++j) {
            vPts_triangle.emplace_back(i, j);
        }
    }

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R << 0, 1,  // 顺时针旋转90°
         1, 0;
    Eigen::Vector2d t(a+d, b + d + g - e);
    for (const auto &pt : vPts_triangle) {
        vPts.push_back(R * pt + t);
    }

    // 去中心化
    int numPts = vPts.size();
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto &pt : vPts) {
        centroid += pt;
    }
    centroid /= numPts;

    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = numPts;
    cloud->height = 1;
    cloud->points.resize(numPts);
    for (int i = 0; i < numPts; ++i) {
        Eigen::Vector2d pt = vPts[i] - centroid;
        auto &point = cloud->points[i];
        point.x = 0.01 * pt.x();
        point.y = 0.01 * pt.y();
        point.z = 0;
    }

    return cloud;
}

PointCloudT::Ptr ConstructComplexArrow(int a, int b, int c, int d, int e, int f, int g, int h, int m, int n, int p) {
    std::vector<Eigen::Vector2d> vPts;
    for (int i = 0; i <= a; ++i) {
        for (int j = 0; j <= c; ++j) {
            vPts.emplace_back(i, j);
        }
    }
    for (int i = a; i <= a+d; ++i) {
        for (int j = i + (b-a); j < i + (b-a+m); ++j) {
            vPts.emplace_back(i, j);
        }
    }

    // 右边的三角形
    {
        std::vector<Eigen::Vector2d> vPts_triangle;
        double k1 = -double(f) / g;
        double k2 = double(f) / h;
        for (int i = 0; i <= g; ++i) {
            for (int j = 0; j < k1 * i + f; ++j) {
                vPts_triangle.emplace_back(i, j);
            }
        }
        for (int i = -h; i < 0; ++i) {
            for (int j = 0; j < k2*i + f; ++j) {
                vPts_triangle.emplace_back(i, j);
            }
        }

        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
        R << 0, 1,  // 顺时针旋转90°
                1, 0;
        Eigen::Vector2d t(a+d, b + d + g - e);
        for (const auto &pt : vPts_triangle) {
            vPts.push_back(R * pt + t);
        }
    }

    // 上面的三角形
    {
        std::vector<Eigen::Vector2d> vPts_triangle;
        double k = -double(p) / (0.5*a + n);
        for (int i = 0; i <= 0.5*a + n; ++i) {
            for (int j = 0; j <= k * i + p; ++j) {
                vPts_triangle.emplace_back(i, j);
                vPts_triangle.emplace_back(-i, j);
            }
        }

        Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
        Eigen::Vector2d t(0.5*a, c);
        for (const auto &pt : vPts_triangle) {
            vPts.push_back(R * pt + t);
        }
    }

    // 去中心化
    int numPts = vPts.size();
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto &pt : vPts) {
        centroid += pt;
    }
    centroid /= numPts;

    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = numPts;
    cloud->height = 1;
    cloud->points.resize(numPts);
    for (int i = 0; i < numPts; ++i) {
        Eigen::Vector2d pt = vPts[i] - centroid;
        auto &point = cloud->points[i];
        point.x = 0.01 * pt.x();
        point.y = 0.01 * pt.y();
        point.z = 0;
    }

    return cloud;
}


PointCloudT::Ptr ConstructDoubleArrow(int a, int b, int c, int d, int e, int f, int g, int h, int m) {
    PointCloudT::Ptr right_cloud(new PointCloudT);
    right_cloud = ConstructTurningArrow(0.5*a, b, b+c, d, e, f, g, h, m);

    int numPts = right_cloud->points.size();
    PointCloudT::Ptr left_cloud(new PointCloudT);
    left_cloud->width = numPts;
    left_cloud->height = 1;
    left_cloud->points.resize(numPts);
    for (int i = 0; i < numPts; ++i) {
        left_cloud->points[i] = right_cloud->points[i];
        left_cloud->points[i].x *= -1;
    }

    *right_cloud += *left_cloud;

    return right_cloud;
}

PointCloudT::Ptr ConstructDoubleComplexArrow(int a, int b, int c, int d, int e, int f, int g, int h, int m, int n, int p) {
    std::vector<Eigen::Vector2d> vPts;
    for (int i = 0; i <= a; ++i) {
        for (int j = 0; j <= c; ++j) {
            vPts.emplace_back(i, j);
        }
    }
    for (int i = a; i <= a+d; ++i) {
        for (int j = i + (b-a); j < i + (b-a+m); ++j) {
            vPts.emplace_back(i, j);
        }
    }

    // 右边的三角形
    {
        std::vector<Eigen::Vector2d> vPts_triangle;
        double k1 = -double(f) / g;
        double k2 = double(f) / h;
        for (int i = 0; i <= g; ++i) {
            for (int j = 0; j < k1 * i + f; ++j) {
                vPts_triangle.emplace_back(i, j);
            }
        }
        for (int i = -h; i < 0; ++i) {
            for (int j = 0; j < k2*i + f; ++j) {
                vPts_triangle.emplace_back(i, j);
            }
        }

        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
        R << 0, 1,  // 顺时针旋转90°
                1, 0;
        Eigen::Vector2d t(a+d, b + d + g - e);
        for (const auto &pt : vPts_triangle) {
            vPts.push_back(R * pt + t);
        }
    }

    // 上面的三角形
    {
        std::vector<Eigen::Vector2d> vPts_triangle;
        double k = -double(p) / (0.5*a + n);
        for (int i = 0; i <= 0.5*a + n; ++i) {
            for (int j = 0; j <= k * i + p; ++j) {
                vPts_triangle.emplace_back(i, j);
                vPts_triangle.emplace_back(-i, j);
            }
        }

        Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
        Eigen::Vector2d t(0.5*a, c);
        for (const auto &pt : vPts_triangle) {
            vPts.push_back(R * pt + t);
        }
    }


    for (int i = -d; i <= 0; ++i) {
        for (int j = -i + b; j < -i + b + m; ++j) {
            vPts.emplace_back(i, j);
        }
    }

    // 左边的三角形
    {
        std::vector<Eigen::Vector2d> vPts_triangle;
        double k1 = double(f) / g;
        double k2 = -double(f) / h;
        for (int i = -g; i <= 0; ++i) {
            for (int j = 0; j < k1 * i + f; ++j) {
                vPts_triangle.emplace_back(i, j);
            }
        }
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < k2*i + f; ++j) {
                vPts_triangle.emplace_back(i, j);
            }
        }

        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
        R << 0, -1,  // 顺时针旋转90°
             1, 0;
        Eigen::Vector2d t(-d, b + d + g - e);
        for (const auto &pt : vPts_triangle) {
            vPts.push_back(R * pt + t);
        }
    }

    // 去中心化
    int numPts = vPts.size();
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto &pt : vPts) {
        centroid += pt;
    }
    centroid /= numPts;

    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = numPts;
    cloud->height = 1;
    cloud->points.resize(numPts);
    for (int i = 0; i < numPts; ++i) {
        Eigen::Vector2d pt = vPts[i] - centroid;
        auto &point = cloud->points[i];
        point.x = 0.01 * pt.x();
        point.y = 0.01 * pt.y();
        point.z = 0;
    }

    return cloud;
}