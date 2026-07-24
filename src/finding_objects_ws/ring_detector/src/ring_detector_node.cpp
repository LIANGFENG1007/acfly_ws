// ============================================================================
//  ring_detector_node.cpp  ── 点云找圆环：求圆环在地图上的水平中心 (x,y)
//
//  场景：场地里立着一个直径≈1m、管径≈3cm 的圆环(随机朝向)，架在支撑杆上。无人机以后
//        要钻圈，当前只需要知道【圆环在地图上的水平位置 (x,y)】。
//
//  输入：/cloud_registered (sensor_msgs/PointCloud2, 世界系 camera_init, Point-LIO 发)。
//  做法：
//    1) ★用 PointCloud2ConstIterator 手动遍历 x/y/z 字段★(与自主探索 exploration_planner 的
//       on_cloud 同款)——不走 pcl::fromROSMsg(它对字段布局挑剔，之前框内 0 点就是它转失败)。
//    2) 3D 包围盒裁剪(xmin/xmax ymin/ymax zmin/zmax 全 -p 可调)：把圆环那一段框出来，
//       ★z 下限要抬到支撑杆顶以上、只留圆环★，否则质心被杆带偏。
//    3) 多帧滑动累积(圆环静止，累积不失真)，攒够点。
//    4) 累积点求【XY 质心】= 圆环水平中心 (x,y)。立着的圆环关于中心对称，XY 平均即圆心水平位置。
//    5) (仅可视化) Eigen PCA 求环平面 + 半径，画拟合圆 Marker，rviz 里确认找对了。
//  输出：
//    /ring_detector/center        geometry_msgs/PointStamped   圆环中心(x,y；z=质心高度)
//    /ring_detector/cropped_cloud sensor_msgs/PointCloud2       累积+裁剪后点云(rviz 看框对不对)
//    /ring_detector/marker        visualization_msgs/Marker     圆心球 + PCA 拟合圆(rviz 看准不准)
//
//  rviz：rviz2 -d <本包>/rviz/ring_detector.rviz(已配 Fixed Frame=camera_init + 三个显示)。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>

#include <atomic>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

using std::placeholders::_1;

class RingDetectorNode : public rclcpp::Node
{
public:
    RingDetectorNode() : Node("ring_detector_node")
    {
        // ---- 参数(全部 -p 可运行期覆盖) ----
        auto dd = [this](const std::string& n, double v) { return this->declare_parameter<double>(n, v); };
        // 3D 包围盒(世界系 camera_init, m)：把圆环框出来。★按场地实际位置改这 6 个★
        box_min_x_ = dd("box_min_x",  1.5);
        box_max_x_ = dd("box_max_x",  3.5);  
        box_min_y_ = dd("box_min_y", -1.0); 
        box_max_y_ = dd("box_max_y",  1.0);
        box_min_z_ = dd("box_min_z",  0.3);    // ★z 下限抬到支撑杆顶以上，只留圆环★
        box_max_z_ = dd("box_max_z",  2.0);    // 上限：盖过圆环顶
        min_points_ = static_cast<int>(std::lround(dd("min_points", 15.0)));  // 少于此不输出(防噪)
        // 多帧累积：细圆环单帧点少，攒最近 N 帧一起处理(圆环静止不失真)。=1 则不累积。
        accum_frames_ = static_cast<int>(std::lround(dd("accum_frames", 30.0)));  // 调整累积帧数
        if (accum_frames_ < 1) accum_frames_ = 1;
        frame_id_  = declare_parameter<std::string>("frame_id", "camera_init");

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", rclcpp::SensorDataQoS(),
            std::bind(&RingDetectorNode::on_cloud, this, _1));

        // 订飞机位姿(拿当前 yaw，算"还需转多少度"对准圆环)
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
            std::bind(&RingDetectorNode::on_odom, this, _1));

        // ★输出话题(不新建，就这一个)：Float64MultiArray data=[x, y, z, yaw_error_deg]★
        //   前三个 = 圆环中心(SLAM系)；第四个 = 飞机还需转多少度(正=左转/逆时针)才正对圆环面。
        center_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/ring_detector/center", 10);
        cloud_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/ring_detector/cropped_cloud", 5);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ring_detector/marker", 5);

        RCLCPP_INFO(get_logger(),
            "ring_detector 已启动。包围盒 x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]，累积 %d 帧，最少点数 %d",
            box_min_x_, box_max_x_, box_min_y_, box_max_y_, box_min_z_, box_max_z_,
            accum_frames_, min_points_);
    }

private:
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ---- 1) 手动遍历 x/y/z 字段 + 包围盒裁剪(与探索节点同款，不用 pcl::fromROSMsg) ----
        std::vector<Eigen::Vector3f> frame_pts;
        frame_pts.reserve(msg->width * msg->height / 4 + 1);

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
            const float x = *it_x, y = *it_y, z = *it_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            // 3D 包围盒
            if (x < box_min_x_ || x > box_max_x_) continue;
            if (y < box_min_y_ || y > box_max_y_) continue;
            if (z < box_min_z_ || z > box_max_z_) continue;
            frame_pts.emplace_back(x, y, z);
        }

        // ---- 2) 多帧滑动累积 ----
        frame_buf_.push_back(std::move(frame_pts));
        while (static_cast<int>(frame_buf_.size()) > accum_frames_) frame_buf_.pop_front();

        std::vector<Eigen::Vector3f> pts;
        for (const auto& f : frame_buf_) pts.insert(pts.end(), f.begin(), f.end());

        // 发累积+裁剪后点云(rviz 看框住了哪些点)——即使点太少也发，方便调包围盒
        publish_cloud(pts, msg->header.stamp);

        if (static_cast<int>(pts.size()) < min_points_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "累积 %d 帧后点数 %zu < %d，未检测到圆环(调包围盒/调大 accum_frames/调小 min_points)",
                static_cast<int>(frame_buf_.size()), pts.size(), min_points_);
            return;
        }

        // ---- 3) 圆拟合：PCA 找环平面 → 投影 2D → 最小二乘圆拟合 → 圆心(3D) ----
        //   ★不用质心当圆心★：激光只扫到圆环几段弧、分布不均，质心会偏向点多一侧；
        //   圆拟合从弧上的点反解出真正的圆心，哪怕只有一段弧也准。输出的 (x,y) 用它。
        Eigen::Vector3d center;   // 拟合圆心(3D)
        double radius = 0.0;
        Eigen::Vector3d e1, e2;   // 环平面内两正交轴(画圆用)
        if (!fit_circle(pts, center, radius, e1, e2)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "圆拟合失败(点太少/退化)，本帧不输出");
            return;
        }

        // ---- 4) 环面法向 + 与飞机的角度差(飞机还需转多少度对准圆环面) ----
        //   环面法向 n = e1×e2(垂直于环面)。取其水平分量算朝向 ring_yaw=atan2(n.y,n.x)。
        Eigen::Vector3d n = e1.cross(e2);
        // 180° 消歧：法向可能指向任一侧，取"从飞机指向圆心"那一侧——飞机对准后正好面向圆环能穿过去。
        Eigen::Vector3d to_ring = center - Eigen::Vector3d(drone_x_, drone_y_, center.z());
        if (n.head<2>().dot(to_ring.head<2>()) < 0.0) n = -n;   // 只用水平分量判同向
        const double ring_yaw = std::atan2(n.y(), n.x());        // 环面朝向(飞机应转到的绝对 yaw)
        double yaw_err = ring_yaw - drone_yaw_;                   // 还需转的角(rad)
        while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;            // 归一到 (-π,π]
        while (yaw_err <= -M_PI) yaw_err += 2.0 * M_PI;
        const double yaw_err_deg = yaw_err * 180.0 / M_PI;

        // ---- 5) 发布 [x, y, z, yaw_error_deg, ring_yaw_deg] (就这一个话题，不新建) ----
        //   前三=圆心(SLAM系)；yaw_error_deg=飞机还需转多少度对准；ring_yaw_deg=环面绝对朝向(SLAM系)，
        //   主控用它算"圆环正前方"停靠点(圆心沿法向退开一段)。
        const double ring_yaw_deg = ring_yaw * 180.0 / M_PI;
        std_msgs::msg::Float64MultiArray out;
        out.data = { center.x(), center.y(), center.z(), yaw_err_deg, ring_yaw_deg };
        center_pub_->publish(out);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "圆环中心: x=%.3f y=%.3f z=%.3f  半径≈%.3fm  还需转 %.1f°  环面朝向 %.1f°  框内 %zu 点",
            center.x(), center.y(), center.z(), radius, yaw_err_deg, ring_yaw_deg, pts.size());

        // ---- 6) 可视化：圆心球 + 拟合圆 + 环面法向箭头 ----
        publish_marker(center, radius, e1, e2, n, msg->header.stamp);
    }

    // 存飞机当前位姿(算角度差用)：位置 + yaw
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        drone_x_ = p.x;
        drone_y_ = p.y;
        // 四元数 → yaw(绕 Z)
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        drone_yaw_ = std::atan2(siny, cosy);
    }

    // 手动构造 PointCloud2 发出去(供 rviz 看裁剪+累积后的点)——同探索节点 publish_obstacle_cloud。
    void publish_cloud(const std::vector<Eigen::Vector3f>& pts, const rclcpp::Time& stamp)
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;
        msg.height = 1;
        msg.width  = static_cast<uint32_t>(pts.size());
        msg.is_dense = true;
        msg.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(pts.size());
        sensor_msgs::PointCloud2Iterator<float> ox(msg, "x"), oy(msg, "y"), oz(msg, "z");
        for (const auto& p : pts) { *ox = p.x(); *oy = p.y(); *oz = p.z(); ++ox; ++oy; ++oz; }
        cloud_pub_->publish(msg);
    }

    // ★圆拟合★：PCA 找环平面 → 点投影到平面 2D → Kåsa 代数最小二乘圆拟合 → 圆心/半径。
    //   出参 center(3D 圆心)/radius/e1,e2(环平面内两正交轴)。返回 false=点太少/退化。
    //   不受"点只在几段弧、分布不均"影响——从弧上点反解真圆心。
    bool fit_circle(const std::vector<Eigen::Vector3f>& pts,
                    Eigen::Vector3d& center, double& radius,
                    Eigen::Vector3d& e1, Eigen::Vector3d& e2)
    {
        const size_t n = pts.size();
        if (n < 3) return false;

        // 均值(作 PCA 原点 + 投影基准)
        Eigen::Vector3d mean(0, 0, 0);
        for (const auto& p : pts) mean += p.cast<double>();
        mean /= static_cast<double>(n);

        // PCA：协方差最大两特征向量 = 环平面内两正交轴(e1,e2)
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto& p : pts) {
            const Eigen::Vector3d d = p.cast<double>() - mean;
            cov += d * d.transpose();
        }
        cov /= static_cast<double>(n);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);   // 特征值升序
        if (es.info() != Eigen::Success) return false;
        e1 = es.eigenvectors().col(2).normalized();   // 最大方向
        e2 = es.eigenvectors().col(1).normalized();   // 次大方向(与 e1 正交，同在环平面)

        // 各点投影到 (e1,e2) 平面得 2D 坐标 (u,v)，相对 mean
        // Kåsa 代数圆拟合：解 [2u 2v 1][a b cc]^T = u²+v²，圆心(a,b)、半径=sqrt(cc+a²+b²)
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d bvec = Eigen::Vector3d::Zero();
        for (const auto& p : pts) {
            const Eigen::Vector3d d = p.cast<double>() - mean;
            const double u = d.dot(e1), v = d.dot(e2);
            const double uu = u * u + v * v;
            Eigen::Vector3d row(2.0 * u, 2.0 * v, 1.0);
            A    += row * row.transpose();
            bvec += row * uu;
        }
        // 解 3x3(用 LDLT，A 对称半正定)
        Eigen::Vector3d sol = A.ldlt().solve(bvec);
        const double a = sol(0), b = sol(1), cc = sol(2);
        const double r2 = cc + a * a + b * b;
        if (!(r2 > 1e-6)) return false;
        radius = std::sqrt(r2);

        // 2D 圆心 (a,b) 变回 3D：mean + a*e1 + b*e2
        center = mean + a * e1 + b * e2;
        return true;
    }

    // 可视化：拟合圆心(红球) + 拟合圆(绿线) + 环面法向(蓝箭头，飞机应朝它对准)。
    void publish_marker(const Eigen::Vector3d& center, double radius,
                        const Eigen::Vector3d& e1, const Eigen::Vector3d& e2,
                        const Eigen::Vector3d& n, const rclcpp::Time& stamp)
    {
        visualization_msgs::msg::Marker sphere;
        sphere.header.stamp = stamp;
        sphere.header.frame_id = frame_id_;
        sphere.ns = "ring_center";
        sphere.id = 0;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = center.x();
        sphere.pose.position.y = center.y();
        sphere.pose.position.z = center.z();
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12;
        sphere.color.r = 1.0f; sphere.color.a = 1.0f;
        marker_pub_->publish(sphere);

        visualization_msgs::msg::Marker circle;
        circle.header.stamp = stamp;
        circle.header.frame_id = frame_id_;
        circle.ns = "ring_fit";
        circle.id = 1;
        circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
        circle.action = visualization_msgs::msg::Marker::ADD;
        circle.scale.x = 0.02;
        circle.color.g = 1.0f; circle.color.a = 1.0f;
        circle.pose.orientation.w = 1.0;
        const int seg = 48;
        for (int k = 0; k <= seg; ++k) {
            const double ang = 2.0 * M_PI * k / seg;
            const Eigen::Vector3d pt = center + radius * (std::cos(ang) * e1 + std::sin(ang) * e2);
            geometry_msgs::msg::Point p;
            p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
            circle.points.push_back(p);
        }
        marker_pub_->publish(circle);

        // 环面法向箭头(蓝)：从圆心沿法向 n(已消歧，指向"飞机穿过去"侧)画一段，飞机对准它即可穿圈。
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp = stamp;
        arrow.header.frame_id = frame_id_;
        arrow.ns = "ring_normal";
        arrow.id = 2;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = 0.03;   // 杆径
        arrow.scale.y = 0.08;   // 箭头径
        arrow.scale.z = 0.0;
        arrow.color.b = 1.0f; arrow.color.a = 1.0f;
        arrow.pose.orientation.w = 1.0;
        {
            const Eigen::Vector3d tip = center + 0.6 * n.normalized();   // 箭头长 0.6m
            geometry_msgs::msg::Point p0, p1;
            p0.x = center.x(); p0.y = center.y(); p0.z = center.z();
            p1.x = tip.x();    p1.y = tip.y();    p1.z = tip.z();
            arrow.points.push_back(p0);
            arrow.points.push_back(p1);
        }
        marker_pub_->publish(arrow);
    }

    // ---- 参数 ----
    double box_min_x_, box_max_x_, box_min_y_, box_max_y_, box_min_z_, box_max_z_;
    int    min_points_ = 15;
    int    accum_frames_ = 10;
    std::string frame_id_ = "camera_init";

    // ---- 多帧累积缓冲：最近 accum_frames_ 帧的(已裁剪)点 ----
    std::deque<std::vector<Eigen::Vector3f>> frame_buf_;

    // ---- 飞机当前位姿(on_odom 写，算角度差用)。原子存，读写跨回调无锁竞争风险小 ----
    std::atomic<double> drone_x_{0.0}, drone_y_{0.0}, drone_yaw_{0.0};

    // ---- ROS ----
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr center_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  marker_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RingDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
