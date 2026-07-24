// ============================================================================
//  pole_detector_node.cpp  ── 点云找杆子：求杆在地图上的水平位置 (x,y)
//
//  场景：场地里立着一根【直径≈3cm 的细杆】(近似竖直)。无人机要知道【杆在地图上的
//        水平位置 (x,y)】(以后绕它/避它/对准它)，当前只需定位。
//
//  ★与 ring_detector 的关系★：同一套点云处理骨架(手遍历+包围盒+多帧累积)，只把
//        "PCA 找环平面 + Kåsa 圆拟合"换成"在【水平 XY 平面】上做 Kåsa 圆拟合"——
//        竖直杆的横截面是圆，框内点投到 XY 反解出的圆心即杆的水平轴心。发布方式照搬。
//
//  输入：/cloud_registered (sensor_msgs/PointCloud2, 世界系 camera_init, Point-LIO 发)。
//  做法：
//    1) ★用 PointCloud2ConstIterator 手动遍历 x/y/z 字段★(与 ring_detector / 探索节点同款)——
//       不走 pcl::fromROSMsg(它对字段布局挑剔，之前框内 0 点就是它转失败)。
//    2) 3D 包围盒裁剪(xmin/xmax ymin/ymax zmin/zmax 全 -p 可调)：把杆那一段框出来，
//       ★z 下限抬到地面以上、上限压到杆顶以下，纵向只留杆身一段★，避免地面/顶端噪声。
//    3) 多帧滑动累积(杆静止，累积不失真)，攒够点(细杆单帧点少)。
//    4) ★在 XY 平面做 Kåsa 圆拟合★(不做 PCA)：竖直杆横截面为圆，框内点投到水平面反解
//       杆【水平轴心 (x,y) + 半径】。比 XY 质心准——激光只扫到杆朝己一侧的弧、分布不均，
//       质心会偏向点多一侧；圆拟合从弧上点反解真圆心，哪怕只有一段弧也准。z 取框内点高度中值。
//    5) 订飞机位姿，算"飞机还需转多少度机头正对杆"(方位角 - 当前 yaw)。杆轴对称无自身朝向，
//       这个 yaw 是【飞机→杆的方位角】对应的转角，不是杆的朝向。
//  输出(照 ring_detector 的话题/类型/风格)：
//    /pole_detector/center        std_msgs/Float64MultiArray   data=[x, y, z, yaw_error_deg, radius]
//                                   x,y=杆水平轴心；z=框内点高度中值；yaw_error_deg=飞机还需转多少度对准杆；
//                                   radius=拟合杆半径(m)。主控绕杆用它算接近半径=radius+standoff。
//    /pole_detector/cropped_cloud sensor_msgs/PointCloud2       累积+裁剪后点云(rviz 看框对不对)
//    /pole_detector/marker        visualization_msgs/Marker    轴心球 + 竖直圆柱轮廓(rviz 看准不准)
//
//  rviz：rviz2 -d <本包>/rviz/pole_detector.rviz(已配 Fixed Frame=camera_init + 三个显示)。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

using std::placeholders::_1;

class PoleDetectorNode : public rclcpp::Node
{
public:
    PoleDetectorNode() : Node("pole_detector_node")
    {
        // ---- 参数(全部 -p 可运行期覆盖) ----
        auto dd = [this](const std::string& n, double v) { return this->declare_parameter<double>(n, v); };
        // 3D 包围盒(世界系 camera_init, m)：把杆框出来。★按场地实际位置改这 6 个★
        box_min_x_ = dd("box_min_x", -0.5);
        box_max_x_ = dd("box_max_x",  4.5);
        box_min_y_ = dd("box_min_y", -0.5);
        box_max_y_ = dd("box_max_y",  4.5);
        box_min_z_ = dd("box_min_z",  0.3);    // ★z 下限抬到地面以上，避开地面点★
        box_max_z_ = dd("box_max_z",  2.0);    // 上限压到杆顶以下，纵向只留杆身一段
        min_points_ = static_cast<int>(std::lround(dd("min_points", 15.0)));  // 少于此不输出(防噪)
        // 多帧累积：细杆单帧点少，攒最近 N 帧一起处理(杆静止不失真)。=1 则不累积。
        accum_frames_ = static_cast<int>(std::lround(dd("accum_frames", 30.0)));  // 调整累积帧数
        if (accum_frames_ < 1) accum_frames_ = 1;
        // 半径合理性上限(m)：拟合半径超此视为拟合到墙/多物、非细杆 → 本帧丢弃(防误检)。
        max_radius_ = dd("max_radius", 0.15);
        frame_id_  = declare_parameter<std::string>("frame_id", "camera_init");

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", rclcpp::SensorDataQoS(),
            std::bind(&PoleDetectorNode::on_cloud, this, _1));

        // 订飞机位姿(拿当前 x/y/yaw，算"还需转多少度"正对杆)
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
            std::bind(&PoleDetectorNode::on_odom, this, _1));

        // ★输出话题(照 ring_detector)：Float64MultiArray data=[x, y, z, yaw_error_deg]★
        //   前三=杆水平轴心(SLAM系)；第四=飞机还需转多少度(正=左转/逆时针)机头正对杆。
        center_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/pole_detector/center", 10);
        cloud_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/pole_detector/cropped_cloud", 5);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/pole_detector/marker", 5);

        RCLCPP_INFO(get_logger(),
            "pole_detector 已启动。包围盒 x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]，累积 %d 帧，最少点数 %d，半径上限 %.2fm",
            box_min_x_, box_max_x_, box_min_y_, box_max_y_, box_min_z_, box_max_z_,
            accum_frames_, min_points_, max_radius_);
    }

private:
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ---- 1) 手动遍历 x/y/z 字段 + 包围盒裁剪(与 ring_detector 同款，不用 pcl::fromROSMsg) ----
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
                "累积 %d 帧后点数 %zu < %d，未检测到杆(调包围盒/调大 accum_frames/调小 min_points)",
                static_cast<int>(frame_buf_.size()), pts.size(), min_points_);
            return;
        }

        // ---- 3) XY 平面圆拟合：竖直杆横截面为圆，框内点投到水平面反解杆水平轴心 (x,y)+半径 ----
        double cx = 0.0, cy = 0.0, radius = 0.0;
        if (!fit_circle_xy(pts, cx, cy, radius)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "XY 圆拟合失败(点太少/退化)，本帧不输出");
            return;
        }
        if (radius > max_radius_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "拟合半径 %.3fm > 上限 %.2fm，疑似非细杆(墙/多物)，本帧丢弃(可调 max_radius)",
                radius, max_radius_);
            return;
        }

        // ---- 4) z 取框内点高度中值(代表杆身中段高度)。杆竖直，z 不影响水平定位，仅作输出参考 ----
        const double cz = median_z(pts);

        // ---- 5) 飞机还需转多少度机头正对杆：方位角(飞机→杆) - 当前 yaw ----
        //   杆轴对称、无自身朝向，故用"飞机指向杆"的方位角。飞机把机头转到该方位即正对杆。
        const double bearing = std::atan2(cy - drone_y_.load(), cx - drone_x_.load());
        double yaw_err = bearing - drone_yaw_.load();
        while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;      // 归一到 (-π,π]
        while (yaw_err <= -M_PI) yaw_err += 2.0 * M_PI;
        const double yaw_err_deg = yaw_err * 180.0 / M_PI;

        // ---- 6) 发布 [x, y, z, yaw_error_deg, radius] (照 ring_detector 的 Float64MultiArray) ----
        std_msgs::msg::Float64MultiArray out;
        out.data = { cx, cy, cz, yaw_err_deg, radius };
        center_pub_->publish(out);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "杆位置: x=%.3f y=%.3f z=%.3f  半径≈%.3fm  还需转 %.1f°  框内 %zu 点",
            cx, cy, cz, radius, yaw_err_deg, pts.size());

        // ---- 7) 可视化：轴心球 + 竖直圆柱轮廓 ----
        publish_marker(cx, cy, cz, radius, msg->header.stamp);
    }

    // 存飞机当前位姿(算方位角用)：位置 + yaw
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

    // 手动构造 PointCloud2 发出去(供 rviz 看裁剪+累积后的点)——同 ring_detector publish_cloud。
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

    // ★XY 平面圆拟合★：只取各点 (x,y)，Kåsa 代数最小二乘圆拟合 → 圆心(cx,cy)/半径。
    //   竖直杆横截面为圆，投到水平面即一段圆弧；从弧上点反解真圆心，不受点分布不均影响。
    //   Kåsa：解 [2x 2y 1][a b cc]^T = x²+y²，圆心(a,b)、半径=sqrt(cc+a²+b²)。返回 false=点太少/退化。
    bool fit_circle_xy(const std::vector<Eigen::Vector3f>& pts,
                       double& cx, double& cy, double& radius)
    {
        const size_t n = pts.size();
        if (n < 3) return false;

        // 用均值平移(数值稳定：坐标绝对值大时直接建正规方程易病态)
        double mx = 0.0, my = 0.0;
        for (const auto& p : pts) { mx += p.x(); my += p.y(); }
        mx /= static_cast<double>(n);
        my /= static_cast<double>(n);

        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d bvec = Eigen::Vector3d::Zero();
        for (const auto& p : pts) {
            const double u = static_cast<double>(p.x()) - mx;
            const double v = static_cast<double>(p.y()) - my;
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
        cx = mx + a;   // 平移回世界系
        cy = my + b;
        return true;
    }

    // 框内点高度中值(对离群点比均值稳)。
    double median_z(const std::vector<Eigen::Vector3f>& pts)
    {
        std::vector<float> zs;
        zs.reserve(pts.size());
        for (const auto& p : pts) zs.push_back(p.z());
        const size_t mid = zs.size() / 2;
        std::nth_element(zs.begin(), zs.begin() + mid, zs.end());
        return static_cast<double>(zs[mid]);
    }

    // 可视化：杆轴心(红球) + 竖直圆柱轮廓(绿线，杆身一段高度)。
    void publish_marker(double cx, double cy, double cz, double radius, const rclcpp::Time& stamp)
    {
        visualization_msgs::msg::Marker sphere;
        sphere.header.stamp = stamp;
        sphere.header.frame_id = frame_id_;
        sphere.ns = "pole_center";
        sphere.id = 0;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = cx;
        sphere.pose.position.y = cy;
        sphere.pose.position.z = cz;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12;
        sphere.color.r = 1.0f; sphere.color.a = 1.0f;
        marker_pub_->publish(sphere);

        // 竖直圆柱轮廓(绿)：在轴心处画一段【框高】范围内、半径=拟合半径的圆柱(用 CYLINDER)。
        visualization_msgs::msg::Marker cyl;
        cyl.header.stamp = stamp;
        cyl.header.frame_id = frame_id_;
        cyl.ns = "pole_fit";
        cyl.id = 1;
        cyl.type = visualization_msgs::msg::Marker::CYLINDER;
        cyl.action = visualization_msgs::msg::Marker::ADD;
        const double h = std::max(0.1, box_max_z_ - box_min_z_);   // 圆柱高 = 框的纵向范围
        cyl.pose.position.x = cx;
        cyl.pose.position.y = cy;
        cyl.pose.position.z = 0.5 * (box_min_z_ + box_max_z_);     // 圆柱中心在框纵向中点
        cyl.pose.orientation.w = 1.0;                              // 竖直(默认沿 Z)
        cyl.scale.x = cyl.scale.y = std::max(0.02, 2.0 * radius);  // 直径
        cyl.scale.z = h;
        cyl.color.g = 1.0f; cyl.color.a = 0.5f;
        marker_pub_->publish(cyl);
    }

    // ---- 参数 ----
    double box_min_x_, box_max_x_, box_min_y_, box_max_y_, box_min_z_, box_max_z_;
    int    min_points_ = 15;
    int    accum_frames_ = 30;
    double max_radius_ = 0.15;
    std::string frame_id_ = "camera_init";

    // ---- 多帧累积缓冲：最近 accum_frames_ 帧的(已裁剪)点 ----
    std::deque<std::vector<Eigen::Vector3f>> frame_buf_;

    // ---- 飞机当前位姿(on_odom 写，算方位角用)。原子存，读写跨回调无锁竞争风险小 ----
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
    rclcpp::spin(std::make_shared<PoleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
