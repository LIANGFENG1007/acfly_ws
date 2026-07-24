// ============================================================================
//  multi_pole_detector_node.cpp  ── 点云找【多根】杆子：逐根定位 + 稳定编号
//
//  场景：场地里立着【多根】直径≈3cm 的细杆(近似竖直)。要同时知道每根杆的水平位置
//        (x,y)，并给每根一个【稳定编号】——同一根物理杆的编号跨帧不变(飞远消失再回来
//        也认回原号)，便于上层按编号引用某根杆。
//
//  ★与 pole_detector(单杆)的关系★：同一套点云骨架(手遍历+包围盒+多帧累积+XY圆拟合)，
//        在"裁剪+累积"之后多做两步：
//          (a) ★欧式聚类(自写，XY 平面)★：把框内点按水平距离分成若干簇，一簇=一根杆。
//          (b) ★持久化 track 编号★：维护一张 {id: (x,y)} 表，每根拟合杆就近认领已知编号
//              (距离<关联阈值→是老杆、更新位置)；认不上才 next_id++ 新建。★已知编号永不释放★，
//              消失的杆回来时按位置认回原号。→ 这就是"编号是死的、不会乱变"的实现。
//
//  输入：/cloud_registered (sensor_msgs/PointCloud2, 世界系 camera_init, Point-LIO 发)。
//  输出：
//    /multi_pole_detector/center        std_msgs/Float64MultiArray
//        data = [id, x, y, z, yaw_err_deg, radius,  id, x, y, z, yaw_err_deg, radius, ...]
//        ★每 6 个一根★(编号在最前)，本帧看到几根就发几根(消失的不发)；按 id 升序排列。
//        id=稳定编号；x,y=杆水平轴心；z=该簇点高度中值；yaw_err_deg=飞机机头还需转多少度
//        正对该杆(方位角-当前yaw)；radius=该杆拟合半径。
//    /multi_pole_detector/cropped_cloud sensor_msgs/PointCloud2    累积+裁剪后点云(rviz 看框)
//    /multi_pole_detector/marker        visualization_msgs/MarkerArray  每根：轴心球 + 编号文字
//
//  rviz：rviz2 -d <本包>/rviz/multi_pole_detector.rviz。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

using std::placeholders::_1;

class MultiPoleDetectorNode : public rclcpp::Node
{
public:
    MultiPoleDetectorNode() : Node("multi_pole_detector_node")
    {
        auto dd = [this](const std::string& n, double v) { return this->declare_parameter<double>(n, v); };
        // 3D 包围盒(世界系 camera_init, m)：把杆群框出来。★按场地实际范围改这 6 个★
        box_min_x_ = dd("box_min_x", -0.5);
        box_max_x_ = dd("box_max_x",  4.0);   
        box_min_y_ = dd("box_min_y", -1.5);
        box_max_y_ = dd("box_max_y",  1.5);
        box_min_z_ = dd("box_min_z",  0.3);    // z 下限抬到地面以上，避开地面点
        box_max_z_ = dd("box_max_z",  2.0);    // 上限压到杆顶以下，纵向只留杆身一段
        min_points_ = static_cast<int>(std::lround(dd("min_points", 10.0)));  // 累积后总点数下限(防空帧)
        accum_frames_ = static_cast<int>(std::lround(dd("accum_frames", 30.0)));
        if (accum_frames_ < 1) accum_frames_ = 1;
        // ── 聚类 ──
        cluster_tol_ = dd("cluster_tol", 0.15);   // 欧式聚类 XY 距离阈值(m)：<此的点算同一根杆。★需 < 杆间距★
        min_cluster_points_ = static_cast<int>(std::lround(dd("min_cluster_points", 8.0)));  // 一簇最少点数(滤噪)
        max_radius_ = dd("max_radius", 0.15);     // 簇拟合半径上限(m)：超此视为非细杆、丢弃
        // ── 编号(持久化 track) ──
        assoc_dist_ = dd("assoc_dist", 0.30);     // track 关联阈值(m)：拟合杆距某已知编号<此→认作它(编号不变)
        track_ema_  = dd("track_ema", 0.3);       // 老编号位置更新低通(0~1)：新观测权重，平滑抖动
        frame_id_  = declare_parameter<std::string>("frame_id", "camera_init");

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", rclcpp::SensorDataQoS(),
            std::bind(&MultiPoleDetectorNode::on_cloud, this, _1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
            std::bind(&MultiPoleDetectorNode::on_odom, this, _1));

        center_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/multi_pole_detector/center", 10);
        cloud_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/multi_pole_detector/cropped_cloud", 5);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/multi_pole_detector/marker", 5);

        RCLCPP_INFO(get_logger(),
            "multi_pole_detector 已启动。盒 x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]，累积%d帧，"
            "聚类阈值%.2fm 簇最少%d点，半径上限%.2fm，编号关联%.2fm",
            box_min_x_, box_max_x_, box_min_y_, box_max_y_, box_min_z_, box_max_z_,
            accum_frames_, cluster_tol_, min_cluster_points_, max_radius_, assoc_dist_);
    }

private:
    // 一根检测到的杆(本帧)：水平轴心 + 半径 + 高度中值 + 分配到的编号
    struct Pole { double x, y, z, radius; int id; };
    // 一个持久化 track：编号 + 最近位置。★永不删除★(编号锁死，消失的杆回来按位置认回)。
    struct Track { int id; double x, y; };

    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ---- 1) 手遍历 x/y/z + 包围盒裁剪(不用 pcl::fromROSMsg) ----
        std::vector<Eigen::Vector3f> frame_pts;
        frame_pts.reserve(msg->width * msg->height / 4 + 1);
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
            const float x = *it_x, y = *it_y, z = *it_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            if (x < box_min_x_ || x > box_max_x_) continue;
            if (y < box_min_y_ || y > box_max_y_) continue;
            if (z < box_min_z_ || z > box_max_z_) continue;
            frame_pts.emplace_back(x, y, z);
        }

        // ---- 2) 多帧累积 ----
        frame_buf_.push_back(std::move(frame_pts));
        while (static_cast<int>(frame_buf_.size()) > accum_frames_) frame_buf_.pop_front();
        std::vector<Eigen::Vector3f> pts;
        for (const auto& f : frame_buf_) pts.insert(pts.end(), f.begin(), f.end());

        publish_cloud(pts, msg->header.stamp);   // 发累积+裁剪点云(即使少也发，方便调盒)

        if (static_cast<int>(pts.size()) < min_points_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "累积后点数 %zu < %d，未检测到杆(调盒/accum_frames/min_points)", pts.size(), min_points_);
            return;
        }

        // ---- 3) 欧式聚类(XY)：把点分成若干簇，一簇=一根杆 ----
        const std::vector<std::vector<int>> clusters = cluster_xy(pts);

        // ---- 4) 逐簇 XY 圆拟合 → 得到本帧所有杆(位置+半径)，滤掉半径过大/拟合失败 ----
        std::vector<Pole> poles;
        for (const auto& idx : clusters) {
            if (static_cast<int>(idx.size()) < min_cluster_points_) continue;
            double cx, cy, r;
            if (!fit_circle_xy(pts, idx, cx, cy, r)) continue;
            if (r > max_radius_) continue;
            Pole p; p.x = cx; p.y = cy; p.z = median_z(pts, idx); p.radius = r; p.id = -1;
            poles.push_back(p);
        }

        if (poles.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "聚类后无有效杆(簇太小/半径超限)。累积 %zu 点、%zu 簇", pts.size(), clusters.size());
            return;
        }

        // ---- 5) ★稳定编号★：每根杆就近认领已知 track 编号；认不上才新建。已知 track 永不删。 ----
        assign_ids(poles);

        // ---- 6) 按 id 升序输出 [id,x,y,z,yaw_err_deg,radius] × N ----
        std::sort(poles.begin(), poles.end(), [](const Pole& a, const Pole& b){ return a.id < b.id; });

        const double dx0 = drone_x_.load(), dy0 = drone_y_.load(), dyaw0 = drone_yaw_.load();
        std_msgs::msg::Float64MultiArray out;
        out.data.reserve(poles.size() * 6);
        for (const auto& p : poles) {
            double yaw_err = std::atan2(p.y - dy0, p.x - dx0) - dyaw0;   // 飞机→该杆方位 - 当前yaw
            while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err <= -M_PI) yaw_err += 2.0 * M_PI;
            out.data.push_back(static_cast<double>(p.id));
            out.data.push_back(p.x);
            out.data.push_back(p.y);
            out.data.push_back(p.z);
            out.data.push_back(yaw_err * 180.0 / M_PI);
            out.data.push_back(p.radius);
        }
        center_pub_->publish(out);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "本帧 %zu 根杆(累计编号 %d 个)。首根: id=%d x=%.2f y=%.2f r=%.3f",
            poles.size(), next_id_, poles.front().id, poles.front().x, poles.front().y, poles.front().radius);

        publish_markers(poles, msg->header.stamp);
    }

    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        drone_x_ = p.x;
        drone_y_ = p.y;
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        drone_yaw_ = std::atan2(siny, cosy);
    }

    // ★欧式聚类(XY)★：BFS 连通。两点 XY 距离 < cluster_tol_ 视为相邻，连通分量=一簇。
    //   杆场景框内点不多(累积后数百)，O(n²) 邻域够用；不引 PCL，纯手写。
    std::vector<std::vector<int>> cluster_xy(const std::vector<Eigen::Vector3f>& pts)
    {
        const int n = static_cast<int>(pts.size());
        std::vector<std::vector<int>> clusters;
        std::vector<char> visited(n, 0);
        const double tol2 = cluster_tol_ * cluster_tol_;
        std::vector<int> stack;
        for (int i = 0; i < n; ++i) {
            if (visited[i]) continue;
            // 从 i 起 BFS/DFS 收一个连通分量
            std::vector<int> comp;
            stack.clear();
            stack.push_back(i);
            visited[i] = 1;
            while (!stack.empty()) {
                const int c = stack.back(); stack.pop_back();
                comp.push_back(c);
                const double cx = pts[c].x(), cy = pts[c].y();
                for (int j = 0; j < n; ++j) {
                    if (visited[j]) continue;
                    const double ddx = pts[j].x() - cx, ddy = pts[j].y() - cy;
                    if (ddx * ddx + ddy * ddy <= tol2) {
                        visited[j] = 1;
                        stack.push_back(j);
                    }
                }
            }
            clusters.push_back(std::move(comp));
        }
        return clusters;
    }

    // ★稳定编号分配★：对本帧每根杆，找 tracks_ 里最近的已知编号，距离<assoc_dist_ 则认领该 id
    //   并低通更新其位置；认不上则 next_id_++ 新建一个 track。★tracks_ 永不删除★。
    //   贪心：把 (杆,track) 所有配对按距离升序，逐个吃(每根杆、每个 track 最多用一次)，避免两根杆抢同一编号。
    void assign_ids(std::vector<Pole>& poles)
    {
        const int np = static_cast<int>(poles.size());
        const int nt = static_cast<int>(tracks_.size());

        // 收集所有 (杆 i, track t, 距离) 且距离<assoc_dist_ 的候选，按距离升序贪心匹配
        struct Pair { double d; int i; int t; };
        std::vector<Pair> pairs;
        for (int i = 0; i < np; ++i)
            for (int t = 0; t < nt; ++t) {
                const double ddx = poles[i].x - tracks_[t].x, ddy = poles[i].y - tracks_[t].y;
                const double d = std::hypot(ddx, ddy);
                if (d < assoc_dist_) pairs.push_back({d, i, t});
            }
        std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b){ return a.d < b.d; });

        std::vector<char> pole_used(np, 0), track_used(nt, 0);
        for (const auto& pr : pairs) {
            if (pole_used[pr.i] || track_used[pr.t]) continue;
            poles[pr.i].id = tracks_[pr.t].id;
            // 低通更新该编号位置(跟随杆的最新观测，平滑抖动)
            tracks_[pr.t].x = track_ema_ * poles[pr.i].x + (1.0 - track_ema_) * tracks_[pr.t].x;
            tracks_[pr.t].y = track_ema_ * poles[pr.i].y + (1.0 - track_ema_) * tracks_[pr.t].y;
            pole_used[pr.i] = 1;
            track_used[pr.t] = 1;
        }
        // 没认领到已知编号的杆 → 新建编号(位置不属于任何已知 track)
        for (int i = 0; i < np; ++i) {
            if (pole_used[i]) continue;
            Track tk; tk.id = next_id_++; tk.x = poles[i].x; tk.y = poles[i].y;
            poles[i].id = tk.id;
            tracks_.push_back(tk);
        }
    }

    // XY 圆拟合(Kåsa)：只取簇内点的 (x,y)。同 pole_detector，均值平移保数值稳定。
    bool fit_circle_xy(const std::vector<Eigen::Vector3f>& pts, const std::vector<int>& idx,
                       double& cx, double& cy, double& radius)
    {
        const size_t n = idx.size();
        if (n < 3) return false;
        double mx = 0.0, my = 0.0;
        for (int i : idx) { mx += pts[i].x(); my += pts[i].y(); }
        mx /= static_cast<double>(n);
        my /= static_cast<double>(n);
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d bvec = Eigen::Vector3d::Zero();
        for (int i : idx) {
            const double u = static_cast<double>(pts[i].x()) - mx;
            const double v = static_cast<double>(pts[i].y()) - my;
            const double uu = u * u + v * v;
            Eigen::Vector3d row(2.0 * u, 2.0 * v, 1.0);
            A    += row * row.transpose();
            bvec += row * uu;
        }
        Eigen::Vector3d sol = A.ldlt().solve(bvec);
        const double a = sol(0), b = sol(1), cc = sol(2);
        const double r2 = cc + a * a + b * b;
        if (!(r2 > 1e-6)) return false;
        radius = std::sqrt(r2);
        cx = mx + a; cy = my + b;
        return true;
    }

    double median_z(const std::vector<Eigen::Vector3f>& pts, const std::vector<int>& idx)
    {
        std::vector<float> zs; zs.reserve(idx.size());
        for (int i : idx) zs.push_back(pts[i].z());
        const size_t mid = zs.size() / 2;
        std::nth_element(zs.begin(), zs.begin() + mid, zs.end());
        return static_cast<double>(zs[mid]);
    }

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

    // 每根杆：轴心球(红) + 编号文字(白，"#id")。用 MarkerArray，一次发全部。
    void publish_markers(const std::vector<Pole>& poles, const rclcpp::Time& stamp)
    {
        visualization_msgs::msg::MarkerArray arr;
        // 先发一个 DELETEALL 清掉上帧残留(消失的杆的 marker 不再刷新)
        {
            visualization_msgs::msg::Marker clr;
            clr.header.frame_id = frame_id_;
            clr.header.stamp = stamp;
            clr.action = visualization_msgs::msg::Marker::DELETEALL;
            arr.markers.push_back(clr);
        }
        for (const auto& p : poles) {
            visualization_msgs::msg::Marker sph;
            sph.header.stamp = stamp;
            sph.header.frame_id = frame_id_;
            sph.ns = "pole_center";
            sph.id = p.id;
            sph.type = visualization_msgs::msg::Marker::SPHERE;
            sph.action = visualization_msgs::msg::Marker::ADD;
            sph.pose.position.x = p.x;
            sph.pose.position.y = p.y;
            sph.pose.position.z = p.z;
            sph.pose.orientation.w = 1.0;
            sph.scale.x = sph.scale.y = sph.scale.z = 0.12;
            sph.color.r = 1.0f; sph.color.a = 1.0f;
            arr.markers.push_back(sph);

            visualization_msgs::msg::Marker txt;
            txt.header.stamp = stamp;
            txt.header.frame_id = frame_id_;
            txt.ns = "pole_id";
            txt.id = p.id;
            txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::msg::Marker::ADD;
            txt.pose.position.x = p.x;
            txt.pose.position.y = p.y;
            txt.pose.position.z = p.z + 0.25;   // 文字浮在球上方
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.2;                   // 字高
            txt.color.r = txt.color.g = txt.color.b = 1.0f; txt.color.a = 1.0f;
            txt.text = "#" + std::to_string(p.id);
            arr.markers.push_back(txt);
        }
        marker_pub_->publish(arr);
    }

    // ---- 参数 ----
    double box_min_x_, box_max_x_, box_min_y_, box_max_y_, box_min_z_, box_max_z_;
    int    min_points_ = 10;
    int    accum_frames_ = 30;
    double cluster_tol_ = 0.15;
    int    min_cluster_points_ = 8;
    double max_radius_ = 0.15;
    double assoc_dist_ = 0.30;
    double track_ema_ = 0.3;
    std::string frame_id_ = "camera_init";

    // ---- 多帧累积缓冲 ----
    std::deque<std::vector<Eigen::Vector3f>> frame_buf_;

    // ---- 持久化编号表(★永不删除★) + 下一个可用编号 ----
    std::vector<Track> tracks_;
    int                next_id_ = 0;

    // ---- 飞机当前位姿(算方位角用) ----
    std::atomic<double> drone_x_{0.0}, drone_y_{0.0}, drone_yaw_{0.0};

    // ---- ROS ----
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr center_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiPoleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
