#include <smpl/debug/marker_conversions.h>

namespace smpl {
namespace visual {

void ConvertMarkerMsgToMarker(const visualization_msgs::Marker& mm, Marker& m)
{
    Eigen::Isometry3d pose(
            Eigen::Translation3d(
                    mm.pose.position.x,
                    mm.pose.position.y,
                    mm.pose.position.z) *
            Eigen::Quaterniond(
                    mm.pose.orientation.w,
                    mm.pose.orientation.x,
                    mm.pose.orientation.y,
                    mm.pose.orientation.z));
    m.pose = pose;

    auto convert_points = [](
        const std::vector<geometry_msgs::Point>& points)
        -> std::vector<Eigen::Vector3d>
    {
        std::vector<Eigen::Vector3d> new_points(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            auto& point = points[i];
            new_points[i] = Eigen::Vector3d(point.x, point.y, point.z);
        }
        return new_points;
    };

    switch (mm.type) {
    case visualization_msgs::Marker::ARROW:
        m.shape = Arrow{ mm.scale.x, mm.scale.y };
        break;
    case visualization_msgs::Marker::CUBE:
        m.shape = Cube{ mm.scale.x, mm.scale.y, mm.scale.z };
        break;
    case visualization_msgs::Marker::SPHERE:
        m.shape = Ellipse{ 0.5 * mm.scale.x, 0.5 * mm.scale.y, 0.5 * mm.scale.z };
        break;
    case visualization_msgs::Marker::CYLINDER:
        m.shape = Cylinder{ 0.5 * mm.scale.x, mm.scale.z };
        break;
    case visualization_msgs::Marker::LINE_STRIP:
        m.shape = LineStrip{ convert_points(mm.points) };
        break;
    case visualization_msgs::Marker::LINE_LIST:
        m.shape = LineList{ convert_points(mm.points) };
        break;
    case visualization_msgs::Marker::CUBE_LIST:
        m.shape = CubeList{ convert_points(mm.points), mm.scale.x };
        break;
    case visualization_msgs::Marker::SPHERE_LIST:
        m.shape = SphereList{ convert_points(mm.points), 0.5 * mm.scale.x };
        break;
    case visualization_msgs::Marker::POINTS:
        m.shape = PointList{ convert_points(mm.points) };
        break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
        m.shape = BillboardText{ mm.text };
        break;
    case visualization_msgs::Marker::MESH_RESOURCE:
        m.shape = MeshResource{ mm.mesh_resource, Eigen::Vector3d(mm.scale.x, mm.scale.y, mm.scale.z) };
        break;
    case visualization_msgs::Marker::TRIANGLE_LIST:
        m.shape = TriangleList{ convert_points(mm.points) };
        break;
    }

    if (!mm.colors.empty()) {
        std::vector<visual::Color> colors;
        colors.reserve(mm.colors.size());
        for (auto& color : mm.colors) {
            colors.push_back(Color{ color.r, color.g, color.b, color.a });
        }
        m.color = std::move(colors);
    } else {
        m.color = visual::Color{ mm.color.r, mm.color.g, mm.color.b, mm.color.a };
    }

    m.frame_id = mm.header.frame_id;
    m.ns = mm.ns;
    m.lifetime = mm.lifetime.toSec();
    m.id = mm.id;
    m.action = (Marker::Action)(int)mm.action;
    if (mm.mesh_use_embedded_materials) {
        m.flags |= Marker::MESH_USE_EMBEDDED_MATERIALS;
    }
    if (mm.frame_locked) {
        m.flags |= Marker::FRAME_LOCKED;
    }
}

void ConvertMarkerToMarkerMsg(
    const Marker& m,
    visualization_msgs::Marker& mm)
{
    auto convert_points = [](
        const std::vector<Eigen::Vector3d>& points)
        -> std::vector<geometry_msgs::Point>
    {
        std::vector<geometry_msgs::Point> new_points(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            auto& point = points[i];
            new_points[i].x = point.x();
            new_points[i].y = point.y();
            new_points[i].z = point.z();
        }
        return new_points;
    };

    mm.header.frame_id = m.frame_id;
    mm.ns = m.ns;
    mm.id = m.id;
    switch (type(m.shape)) {
    case SHAPE_EMPTY:
        mm.type = visualization_msgs::Marker::SPHERE;
        mm.scale.x = mm.scale.y = mm.scale.z = 0.1;
        break;
    case SHAPE_ARROW:
        mm.type = visualization_msgs::Marker::ARROW;
        mm.scale.x = boost::get<Arrow>(m.shape).length;
        mm.scale.y = mm.scale.z = boost::get<Arrow>(m.shape).width;
        break;
    case SHAPE_CUBE:
        mm.type = visualization_msgs::Marker::CUBE;
        mm.scale.x = boost::get<Cube>(m.shape).length;
        mm.scale.y = boost::get<Cube>(m.shape).width;
        mm.scale.z = boost::get<Cube>(m.shape).height;
        break;
    case SHAPE_SPHERE:
        mm.type = visualization_msgs::Marker::SPHERE;
        mm.scale.x = mm.scale.y = mm.scale.z = 2.0 * boost::get<Sphere>(m.shape).radius;
        break;
    case SHAPE_ELLIPSE:
        mm.type = visualization_msgs::Marker::SPHERE;
        mm.scale.x = 2.0 * boost::get<Ellipse>(m.shape).axis_x;
        mm.scale.y = 2.0 * boost::get<Ellipse>(m.shape).axis_y;
        mm.scale.z = 2.0 * boost::get<Ellipse>(m.shape).axis_z;
        break;
    case SHAPE_CYLINDER:
        mm.scale.x = 2.0 * boost::get<Cylinder>(m.shape).radius;
        mm.scale.y = 2.0 * boost::get<Cylinder>(m.shape).radius;
        mm.scale.z = boost::get<Cylinder>(m.shape).height;
        mm.type = visualization_msgs::Marker::CYLINDER;
        break;
    case SHAPE_LINE_LIST:
        mm.type = visualization_msgs::Marker::LINE_LIST;
        mm.scale.x = 0.02; // line width
        mm.points = convert_points(boost::get<LineList>(m.shape).points);
        break;
    case SHAPE_LINE_STRIP:
        mm.type = visualization_msgs::Marker::LINE_STRIP;
        mm.scale.x = 0.02; // line width
        mm.points = convert_points(boost::get<LineStrip>(m.shape).points);
        break;
    case SHAPE_CUBE_LIST:
        mm.type = visualization_msgs::Marker::CUBE_LIST;
        mm.scale.x = mm.scale.y = mm.scale.z = boost::get<CubeList>(m.shape).size;
        mm.points = convert_points(boost::get<CubeList>(m.shape).points);
        break;
    case SHAPE_SPHERE_LIST:
        mm.type = visualization_msgs::Marker::SPHERE_LIST;
        mm.scale.x = mm.scale.y = mm.scale.z = 2.0 * boost::get<SphereList>(m.shape).radius;
        mm.points = convert_points(boost::get<SphereList>(m.shape).points);
        break;
    case SHAPE_POINT_LIST:
        mm.type = visualization_msgs::Marker::POINTS;
        mm.scale.x = mm.scale.y = 0.02; // point width and height
        mm.points = convert_points(boost::get<PointList>(m.shape).points);
        break;
    case SHAPE_BILLBOARD_TEXT:
        mm.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mm.text = boost::get<BillboardText>(m.shape).text;
        mm.scale.z = 0.1; // height of 'A'
        break;
    case SHAPE_MESH_RESOURCE:
        mm.type = visualization_msgs::Marker::MESH_RESOURCE;
        mm.mesh_resource = boost::get<MeshResource>(m.shape).uri;
        mm.scale.x = boost::get<MeshResource>(m.shape).scale.x();
        mm.scale.y = boost::get<MeshResource>(m.shape).scale.y();
        mm.scale.z = boost::get<MeshResource>(m.shape).scale.z();
        break;
    case SHAPE_TRIANGLE_LIST:
        mm.type = visualization_msgs::Marker::TRIANGLE_LIST;
        mm.scale.x = mm.scale.y = mm.scale.z = 1.0;
        mm.points = convert_points(boost::get<TriangleList>(m.shape).vertices);
        break;
    default:
        break;
    }

    switch (m.action) {
    case Marker::ACTION_ADD:
        mm.action = visualization_msgs::Marker::ADD;
        break;
    case Marker::ACTION_MODIFY:
        mm.action = visualization_msgs::Marker::MODIFY;
        break;
    case Marker::ACTION_DELETE:
        mm.action = visualization_msgs::Marker::DELETE;
        break;
    case Marker::ACTION_DELETE_ALL:
        mm.action = 3;
        break;
    }

    mm.pose.position.x = m.pose.position.x();
    mm.pose.position.y = m.pose.position.y();
    mm.pose.position.z = m.pose.position.z();
    mm.pose.orientation.w = m.pose.orientation.w();
    mm.pose.orientation.x = m.pose.orientation.x();
    mm.pose.orientation.y = m.pose.orientation.y();
    mm.pose.orientation.z = m.pose.orientation.z();

    if (m.color.which() == 0) {
        auto& color = boost::get<Color>(m.color);
        mm.color.r = color.r;
        mm.color.g = color.g;
        mm.color.b = color.b;
        mm.color.a = color.a;
    } else {
        auto& colors = boost::get<std::vector<Color>>(m.color);
        mm.colors.resize(colors.size());
        for (size_t i = 0; i < colors.size(); ++i) {
            std_msgs::ColorRGBA c;
            mm.colors[i].r = colors[i].r;
            mm.colors[i].g = colors[i].g;
            mm.colors[i].b = colors[i].b;
            mm.colors[i].a = colors[i].a;
        }
    }

    mm.frame_locked = (m.flags & Marker::FRAME_LOCKED);
    mm.mesh_use_embedded_materials = (m.flags & Marker::MESH_USE_EMBEDDED_MATERIALS);
}

auto ConvertMarkersToMarkerArray(const std::vector<Marker>& markers)
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray ma;
    ma.markers.reserve(markers.size());

    for (auto& marker : markers) {
        visualization_msgs::Marker m;
        ConvertMarkerToMarkerMsg(marker, m);
        ma.markers.push_back(std::move(m));
    }

    return ma;
}

auto ConvertMarkerArrayToMarkers(const visualization_msgs::MarkerArray& ma)
    -> std::vector<smpl::visual::Marker>
{
    std::vector<smpl::visual::Marker> markers;
    markers.reserve(ma.markers.size());

    for (auto& m : ma.markers) {
        smpl::visual::Marker marker;
        ConvertMarkerMsgToMarker(m, marker);
        markers.push_back(std::move(marker));
    }

    return markers;
}

} // namespace visual
} // namespace smpl
