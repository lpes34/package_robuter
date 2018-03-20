#include "fanuc_post_processor_library/fanuc_post_processor_library.hpp"

/** @brief Transforms radians into degrees */
#define RAD2DEG(x) ((x)*57.29578)

FanucPostProcessor::FanucPostProcessor() :
    program_name_("ros"),
    program_comment_(""),
    permissions_(READ_WRITE),
    default_group_("1,*,*,*,*"),
    applicative_(""),
    user_frame_(0),
    user_tool_(1),
    line_numbers_(false)
{
}

FanucPostProcessor::~FanucPostProcessor()
{
}

bool FanucPostProcessor::generateProgram(std::string &output_program)
{
  output_program.clear();
  std::string tmp;

  // PROG
  output_program.append("/PROG ");
  output_program.append(program_name_);
  output_program.append("\n");

  // /ATTR
  output_program.append("/ATTR");
  output_program.append("\n");
  output_program.append("COMMENT = \"" + program_comment_ + "\";");
  output_program.append("\n");
  output_program.append("PROTECT = " + permissionsToString(permissions_) + ";");
  output_program.append("\n");

  if (default_group_ != "")
  {
    output_program.append("DEFAULT_GROUP = "+ default_group_ + ";");
    output_program.append("\n");
  }

  // APPL
  if (applicative_ != "")
  {
    output_program.append("/APPL\n");
    output_program.append("  "+applicative_+";");
    output_program.append("\n");
  }

  // MN
  output_program.append("/MN");
  output_program.append("\n");
  for (std::vector<std::string>::const_iterator iter(lines_.begin()); iter != lines_.end(); ++iter)
  {
    output_program.append(*iter);
    output_program.append("\n");
  }

  // POS
  output_program.append("/POS");
  output_program.append("\n");

  typedef std::map<unsigned, Eigen::Isometry3d, std::less<unsigned>,
      Eigen::aligned_allocator<std::pair<const unsigned, Eigen::Isometry3d> > > Poses;

  for (Poses::const_iterator iter (poses_.begin()); iter != poses_.end(); ++iter)
  {
    std::string pose_block;
    pose_block.append("P[" + boost::lexical_cast<std::string>(iter->first) + "]{\n");
    pose_block.append("   GP1:\n");
    // FIXME: Very bad, the UF and UT should be recorded per pose!
    // FIXME: Robot configuration is hard-coded to NUT,0,0,0!
    pose_block.append("\tUF : " + boost::lexical_cast<std::string>(user_frame_) + ", UT : " + boost::lexical_cast<std::string>(user_tool_) +
                      ",         CONFIG : '"+"N U T, 0, 0, 0',\n");

    const Eigen::Vector3d euler_angles (iter->second.linear().eulerAngles(0, 1, 2));

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << iter->second.translation()[0] * 1000.0;
    std::string x = ss.str();
    ss.str("");
    ss.clear();
    ss << iter->second.translation()[1] * 1000.0;
    std::string y = ss.str();
    ss.str("");
    ss.clear();
    ss << iter->second.translation()[2] * 1000.0;
    std::string z = ss.str();
    ss.str("");
    ss.clear();
    ss << RAD2DEG(euler_angles[0]);
    std::string w = ss.str();
    ss.str("");
    ss.clear();
    ss << RAD2DEG(euler_angles[1]);
    std::string p = ss.str();
    ss.str("");
    ss.clear();
    ss << RAD2DEG(euler_angles[2]);
    std::string r = ss.str();

    pose_block.append("\tX = " + x + " mm, Y = " + y + " mm, Z = " + z + " mm,\n");
    pose_block.append("\tW = " + w + " deg, P = " + p + " deg, R = " + r + " deg\n");

    if (default_group_ == "1,1,*,*,*")
    { // FIXME: External groups must be correctly handled
      pose_block.append("   GP2:\n");
      // FIXME: Very bad, the UF and UT should be recorded per pose!
      // FIXME: Robot configuration is hard-coded to NUT,0,0,0!
      pose_block.append(
          "\tUF : " + boost::lexical_cast<std::string>(user_frame_) + ", UT : "
              + boost::lexical_cast<std::string>(user_tool_) + ",\n");
      pose_block.append("\tJ1= 0.000 deg, J2= 0.000 deg\n");
    }

    pose_block.append("};\n");
    output_program.append(pose_block);
  }
  // END
  output_program.append("/END\n");
  return true;
}

void FanucPostProcessor::clearProgram()
{
  lines_.clear();
  poses_.clear();
  labels_id_.clear();
}

void FanucPostProcessor::useLineNumbers(bool line_numbers)
{
  line_numbers_ = line_numbers;
}

bool FanucPostProcessor::uploadToFtp(const std::string ip_address, const std::string port_number,
                                     const std::string username, const std::string password)
{


ROS_WARN_STREAM("ip_address =="<<ip_address);
ROS_WARN_STREAM("port_number =="<<port_number);
ROS_WARN_STREAM("username =="<<username);
ROS_WARN_STREAM("password =="<<password);

  // Generate program
  std::string tp_program;
  if (!generateProgram(tp_program))
  {
    ROS_ERROR_STREAM("FanucPostProcessor::uploadToFtp: Could not generate program");
    return false;
  }

  // Write program on disk
  std::ofstream myfile;
  myfile.open ("/tmp/" + program_name_ + ".ls");
  if (!myfile.is_open())
  {
    ROS_ERROR_STREAM("FanucPostProcessor::uploadToFtp: Could not open /tmp/" << program_name_ << ".ls for writing");
    return false;
  }
  myfile << tp_program;
  myfile.close();

  // Upload to the robot
  ROS_INFO_STREAM("Trying to upload "+ program_name_ + ".ls at "+ ip_address);
  try
  {
    curlite::Easy easy;
    easy.set(CURLOPT_URL, "ftp://@" + ip_address + ":" + port_number + "/\\"+ program_name_ + ".ls");
    easy.set(CURLOPT_USERNAME, username);
    easy.set(CURLOPT_PASSWORD, password);
    easy.set(CURLOPT_UPLOAD, true);

    // Open input file stream
    std::ifstream ifs("/tmp/" + program_name_ + ".ls", std::ios::binary);
    ifs >> easy;

    double totalSecs = easy.getInfo<double>(CURLINFO_TOTAL_TIME);
    ROS_INFO_STREAM("Upload time: " << totalSecs << " s");
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("FanucPostProcessor::uploadToFtp: Got an exception: " << e.what());
    return false;
  }

  return true;
}

bool FanucPostProcessor::setProgramName(const std::string name)
{
  // FIXME: What characters are allowed/forbidden?
  // FIXME: Must be upper-cased?
  program_name_ = boost::to_upper_copy<std::string>(name);
  return true;
}

bool FanucPostProcessor::setProgramComment(const std::string comment)
{
  // FIXME: What characters are allowed/forbidden?
  program_comment_ = comment;
  return true;
}

bool FanucPostProcessor::setPermissions(const FanucPostProcessor::Permissions perms)
{
  permissions_ = perms;
  return true;
}

bool FanucPostProcessor::setDefaultgroup(const std::string default_group)
{
  default_group_ = default_group;
  return true;
}

bool FanucPostProcessor::setApplicative(const std::string appl)
{
  applicative_ = appl;
  return true;
}

bool FanucPostProcessor::appendPoseFine(const FanucPostProcessor::MovementType move_type, const Eigen::Isometry3d &pose,
                                        const unsigned pose_id, const unsigned speed,
                                        const FanucPostProcessor::SpeedType speed_type, const std::string option)
{
  if (pose_id == 0)
    return false;

  if (move_type == JOINT)
  {
    if (speed_type != PERCENTAGE)
      return false;

    if (speed > 100)
      return false;
  }
  else if (move_type == LINEAR)
  {
    if (speed_type == PERCENTAGE)
      return false;
  }
  else
    return false;

  const std::string line_header(generateLineNumber());

  // Check if pose id is free
  if (poses_.find(pose_id) != poses_.end())
    return false;

  // Add pose to the map
  poses_[pose_id] = pose;

  const std::string p_id(boost::lexical_cast<std::string>(pose_id));
  const std::string move_speed(boost::lexical_cast<std::string>(speed));
  const std::string movement(movementTypeToString(move_type));
  const std::string speed_unit(speedTypeToString(speed_type));

  lines_.push_back(line_header + movement + " P[" + p_id + "] " + move_speed + speed_unit + " FINE " + option + ";");
  return true;
}

bool FanucPostProcessor::appendPoseFine(const FanucPostProcessor::MovementType move_type, const unsigned pose_id,
                                        const unsigned speed, const FanucPostProcessor::SpeedType speed_type,
                                        const std::string option)
{
  if (pose_id == 0)
    return false;

  if (move_type == JOINT)
  {
    if (speed_type != PERCENTAGE)
      return false;

    if (speed > 100)
      return false;
  }
  else if (move_type == LINEAR)
  {
    if (speed_type == PERCENTAGE)
      return false;
  }
  else
    return false;

  const std::string line_header(generateLineNumber());

  // Check if pose id exists
  if (poses_.find(pose_id) == poses_.end())
    return false;

  const std::string p_id(boost::lexical_cast<std::string>(pose_id));
  const std::string move_speed(boost::lexical_cast<std::string>(speed));
  const std::string movement(movementTypeToString(move_type));
  const std::string speed_unit(speedTypeToString(speed_type));

  lines_.push_back(line_header + movement + " P[" + p_id + "] " + move_speed + speed_unit + " FINE " + option + ";");
  return true;
}

bool FanucPostProcessor::appendPoseCNT(const FanucPostProcessor::MovementType move_type, const Eigen::Isometry3d &pose,
                                       const unsigned pose_id, const unsigned speed,
                                       const FanucPostProcessor::SpeedType speed_type, const unsigned cnt,
                                       const std::string option)
{
  if (pose_id == 0)
    return false;

  if (move_type == JOINT)
  {
    if (speed_type != PERCENTAGE)
      return false;

    if (speed > 100)
      return false;
  }
  else if (move_type == LINEAR)
  {
    if (speed_type == PERCENTAGE)
      return false;
  }
  else
    return false;

  if (cnt > 100)
    return false;

  const std::string line_header(generateLineNumber());

  // Check if pose id is free
  if (poses_.find(pose_id) != poses_.end())
    return false;

  // Add pose to the map
  poses_[pose_id] = pose;

  const std::string p_id(boost::lexical_cast<std::string>(pose_id));
  const std::string move_speed(boost::lexical_cast<std::string>(speed));
  const std::string movement(movementTypeToString(move_type));
  const std::string speed_unit(speedTypeToString(speed_type));
  const std::string cnt_value(boost::lexical_cast<std::string>(cnt));

  lines_.push_back(line_header + movement + " P[" + p_id + "] " + move_speed + speed_unit + " CNT" + cnt_value + " " + option + ";");
  return true;
}

bool FanucPostProcessor::appendPoseCNT(const FanucPostProcessor::MovementType move_type, const unsigned pose_id,
                                       const unsigned speed, const FanucPostProcessor::SpeedType speed_type,
                                       const unsigned cnt, const std::string option)
{
  if (pose_id == 0)
    return false;

  if (move_type == JOINT)
  {
    if (speed_type != PERCENTAGE)
      return false;

    if (speed > 100)
      return false;
  }
  else if (move_type == LINEAR)
  {
    if (speed_type == PERCENTAGE)
      return false;
  }
  else
    return false;

  if (cnt > 100)
    return false;

  const std::string line_header(generateLineNumber());

  // Check if pose id exists
  if (poses_.find(pose_id) == poses_.end())
    return false;

  const std::string p_id(boost::lexical_cast<std::string>(pose_id));
  const std::string move_speed(boost::lexical_cast<std::string>(speed));
  const std::string movement(movementTypeToString(move_type));
  const std::string speed_unit(speedTypeToString(speed_type));
  const std::string cnt_value(boost::lexical_cast<std::string>(cnt));

  lines_.push_back(line_header + movement + " P[" + p_id + "] " + move_speed + speed_unit + " CNT" + cnt_value + " " + option + ";");
  return true;
}

void FanucPostProcessor::appendComment(const std::string comment)
{
  const std::string line_header(generateLineNumber());
  lines_.push_back(line_header + "  !" + comment + ";");
}

void FanucPostProcessor::appendEmptyLine()
{
  const std::string line_header(generateLineNumber());
  lines_.push_back(line_header + "  ;");
}

void FanucPostProcessor::appendDigitalOutput(const unsigned digital_out_id, const bool state)
{
  const std::string line_header(generateLineNumber());
  const std::string DO_id(boost::lexical_cast<std::string>(digital_out_id));
  const std::string on_off(state == true ? "ON" : "OFF");
  lines_.push_back(line_header + "  DO[" + DO_id + "]=" + on_off + ";");
}

bool FanucPostProcessor::appendDigitalOutput(const unsigned digital_out_id, const double pulse_time)
{
  if (pulse_time < 0)
    return false;

  const std::string line_header(generateLineNumber());
  const std::string DO_id(boost::lexical_cast<std::string>(digital_out_id));
  std::locale::global(std::locale::classic()); // To ensure . as a decimal separator
  const std::string time(boost::lexical_cast<std::string>(pulse_time));
  lines_.push_back(line_header + "  DO[" + DO_id + "]=PULSE," + time + "sec;");
  return true;
}

bool FanucPostProcessor::appendWait(const double time)
{
  if (time < 0)
    return false;

  const std::string line_header(generateLineNumber());
  std::locale::global(std::locale::classic()); // To ensure . as a decimal separator
  const std::string duration(boost::lexical_cast<std::string>(time));
  lines_.push_back(line_header + "  WAIT  " + duration + "(sec);");
  return true;
}

void FanucPostProcessor::appendWait(const unsigned digital_in_id, bool state)
{
  const std::string line_header(generateLineNumber());
  const std::string DI_id(boost::lexical_cast<std::string>(digital_in_id));
  const std::string DI_state (state ? "ON" : "OFF");
  lines_.push_back(line_header + "  WAIT DI[" + DI_id + "]=" + DI_state + ";");
}

void FanucPostProcessor::appendUFrame(const unsigned uf_id)
{
  const std::string line_header(generateLineNumber());
  user_frame_ = uf_id;
  const std::string uframe(boost::lexical_cast<std::string>(uf_id));
  lines_.push_back(line_header + "  UFRAME_NUM=" + uframe + ";");
}

void FanucPostProcessor::appendUTool(const unsigned ut_id)
{
  const std::string line_header(generateLineNumber());
  user_tool_ = ut_id;
  const std::string utool(boost::lexical_cast<std::string>(ut_id));
  lines_.push_back(line_header + "  UTOOL_NUM=" + utool + ";");
}

void FanucPostProcessor::appendGroupOutput(const unsigned id, const unsigned value)
{
  const std::string line_header(generateLineNumber());
  const std::string group_id(boost::lexical_cast<std::string>(id));
  const std::string group_value(boost::lexical_cast<std::string>(value));
  lines_.push_back(line_header + "  GO[" + group_id + "]=" + group_value + ";");
}

void FanucPostProcessor::appendSetRegister(const unsigned r_id, const double value)
{
  const std::string line_header(generateLineNumber());
  const std::string id(boost::lexical_cast<std::string>(r_id));
  std::locale::global(std::locale::classic()); // To ensure . as a decimal separator
  const std::string r_value(boost::lexical_cast<std::string>(value));
  lines_.push_back(line_header + "  R[" + id + "]=" + r_value + ";");
}

bool FanucPostProcessor::appendRun(const std::string program_name)
{
  // FIXME: What characters are allowed/forbidden?
  // FIXME: Must be upper-cased?
  const std::string line_header(generateLineNumber());
  const std::string name(boost::to_upper_copy<std::string>(program_name));
  lines_.push_back(line_header + "  RUN " + name + ";");
  return true;
}

bool FanucPostProcessor::appendLabel(const unsigned id)
{
  // Search if label is used, if not return false
  for (std::vector<unsigned>::iterator iter (labels_id_.begin()); iter != labels_id_.end(); ++iter)
    if (*iter == id)
      return false;
  labels_id_.push_back(id); // Label "id" is now available in appendJumpLabel

  const std::string line_header(generateLineNumber());
  const std::string lbl_id(boost::lexical_cast<std::string>(id));
  lines_.push_back(line_header + "  LBL[" + lbl_id + "];");
  return true;
}

bool FanucPostProcessor::appendJumpLabel(const unsigned id)
{
  // Search if label is available, if not return false
  bool available = false;
  for (std::vector<unsigned>::iterator iter (labels_id_.begin()); iter != labels_id_.end(); ++iter)
    if (*iter == id)
      available = true;

  if (!available)
    return false;

  const std::string line_header(generateLineNumber());
  const std::string lbl_id(boost::lexical_cast<std::string>(id));
  lines_.push_back(line_header + "  JMP LBL[" + lbl_id + "];");
  return true;
}

void FanucPostProcessor::appendDataMonitorStart(const unsigned id)
{
  const std::string line_header(generateLineNumber());
  const std::string datamon_id(boost::lexical_cast<std::string>(id));
  lines_.push_back(line_header + "  Sample Start[" + datamon_id + "];");
}

void FanucPostProcessor::appendDataMonitorStop()
{
  const std::string line_header(generateLineNumber());
  lines_.push_back(line_header + "  Sample End;");
}

std::string FanucPostProcessor::generateLineNumber()
{
  if (!line_numbers_)
    return ":";

  if (lines_.size() > 9999)
    return ""; // FIXME: Error here!

  std::string line_header(boost::lexical_cast<std::string>(lines_.size() + 1));
  line_header.append(":");

  unsigned characters_to_be_added(5 - line_header.size());
  for (unsigned i(0); i < characters_to_be_added; ++i)
    line_header.insert(0, " ");

  return line_header;
}
