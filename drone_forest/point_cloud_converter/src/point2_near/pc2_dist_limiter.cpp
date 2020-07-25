#include "pc2_dist_limiter.h"

using namespace point_cloud_converter;

PC2DistLimiter::PC2DistLimiter(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  nh_private.param("frame_id", frame_id_, std::string("map"));
  nh_private.param("radius", limit_radius_, 3.0);
  nh_private.param("rate", rate_, 2.0);
  
//  delay_ = 1.0/rate_
//  prev_time_ = ros::Time::now().toSec();

  robot_pos_sub_ = nh.subscribe("/mavros/local_position/pose", 100, &PC2DistLimiter::robotPosCallback, this);
  raw_marker_arr_sub_ = nh.subscribe("/occupied_cells_vis_array", 100, &PC2DistLimiter::rawMarkerArrCallback, this);
  limeted_pc2_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/point2/near", 100);
}

PC2DistLimiter::~PC2DistLimiter()
{
}

double PC2DistLimiter::rate()
{
  return rate_;
}

double PC2DistLimiter::limitRadius()
{
  return limit_radius_;
}

std::string PC2DistLimiter::farmeId()
{
  return frame_id_;
}

void PC2DistLimiter::process()
{
//delay
/*
   if(ros::Time::now().toSec()-prev_time > delay)
    {
        prev_time =ros::Time::now().toSec();
    }
    else
    {return;}
*/
  filterPointCloud();
  sensor_msgs::PointCloud2 filtered_pc2_msg = createPointCloud2();
  pubFilteredPointCloud2(filtered_pc2_msg);
}


void PC2DistLimiter::robotPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  robot_point_.x = static_cast<float>(msg->pose.position.x);
  robot_point_.y = static_cast<float>(msg->pose.position.y);
  robot_point_.z = static_cast<float>(msg->pose.position.z);
}

void PC2DistLimiter::rawMarkerArrCallback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
  raw_pc_points_.clear();
  for(auto marker : msg->markers)
    for(auto point : marker.points)
    {
      GeometryPoint p;
      p.x = static_cast<float>(point.x);
      p.y = static_cast<float>(point.y);
      p.z = static_cast<float>(point.z);
      raw_pc_points_.push_back(p);
    }
}

void PC2DistLimiter::filterPointCloud()
{
  if(raw_pc_points_.empty())
    return;

  filtered_pc_points_.clear();
  for(auto point : raw_pc_points_)
  {
    double dist = getDistBetween<GeometryPoint>(robot_point_, point);
    if(dist <= limit_radius_)
      filtered_pc_points_.push_back(point);
  }
}

template <typename T>
double PC2DistLimiter::getDistBetween(const T &point_1, const T &point_2)
{
  return sqrt( fabs((point_2.x-point_1.x) * (point_2.x-point_1.x) +
                    (point_2.y-point_1.y) * (point_2.y-point_1.y) +
                    (point_2.z-point_1.z) * (point_2.z-point_1.z)));
}

sensor_msgs::PointCloud2 PC2DistLimiter::createPointCloud2()
{
  sensor_msgs::PointCloud2 pc;
  pc.header = std_msgs::Header();
  pc.header.frame_id = frame_id_;
  pc.header.stamp = ros::Time::now();

  unsigned int point_size = sizeof(filtered_pc_points_.front().x);
  unsigned int points_count = static_cast<unsigned int>(filtered_pc_points_.size());

  sensor_msgs::PointField point_field;
  point_field.datatype = sensor_msgs::PointField::FLOAT32; point_field.count = 1;
  point_field.name = "x"; point_field.offset = point_size*0; pc.fields.push_back(point_field);
  point_field.name = "y"; point_field.offset = point_size*1; pc.fields.push_back(point_field);
  point_field.name = "z"; point_field.offset = point_size*2; pc.fields.push_back(point_field);

  pc.is_dense = false;
  pc.is_bigendian = false;

  pc.height = 1;
  pc.width = points_count;

  pc.point_step = static_cast<unsigned int>(point_size*pc.fields.size());
  pc.row_step = pc.point_step*points_count;
  copyPointVectorToByteVector(filtered_pc_points_, pc.data);

  return pc;
}

void PC2DistLimiter::copyPointVectorToByteVector(const GeometryPointsVec& p_vec, std::vector<uint8_t>& buffer)
{
  for(auto p : p_vec)
  {
    copyToByteVector(&p.x, buffer);
    copyToByteVector(&p.y, buffer);
    copyToByteVector(&p.z, buffer);
  }
}

template <typename T>
void PC2DistLimiter::copyToByteVector(T* val, std::vector<uint8_t>& buffer)
{
  std::vector<uint8_t> buf;
  uint8_t* ibegin = reinterpret_cast<uint8_t*>(val);
  auto size = (sizeof(*val)/sizeof(uint8_t));
  buf.assign(ibegin, ibegin + size);
  buffer.insert(buffer.end(), buf.begin(), buf.end());
}

void PC2DistLimiter::pubFilteredPointCloud2(const sensor_msgs::PointCloud2& msg)
{
  if(!pc2MsgIsEmpty(msg))
    limeted_pc2_pub_.publish(msg);
}

bool PC2DistLimiter::pc2MsgIsEmpty(const sensor_msgs::PointCloud2 &msg)
{
  return (msg.width == 0);
}







