#include "brightdrive_localization/pcd_io.hpp"

#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace brightdrive {

bool file_exists(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  return stream.good();
}

PcdCloud read_pcd_xyz(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open pcd: " + path);
  }

  PcdCloud cloud;
  std::size_t point_count = 0;
  bool saw_points = false;
  std::string line;
  std::streampos data_offset = 0;

  while (std::getline(stream, line)) {
    std::istringstream tokens(line);
    std::string key;
    tokens >> key;
    if (key == "FIELDS") {
      std::vector<std::string> fields;
      std::string field;
      while (tokens >> field) {
        fields.push_back(field);
      }
      if (fields.size() != 3 || fields[0] != "x" || fields[1] != "y" || fields[2] != "z") {
        throw std::runtime_error("pcd must contain exactly x y z fields: " + path);
      }
    } else if (key == "SIZE") {
      int a = 0, b = 0, c = 0;
      tokens >> a >> b >> c;
      if (a != 4 || b != 4 || c != 4) {
        throw std::runtime_error("pcd xyz fields must be float32: " + path);
      }
    } else if (key == "TYPE") {
      std::string a, b, c;
      tokens >> a >> b >> c;
      if (a != "F" || b != "F" || c != "F") {
        throw std::runtime_error("pcd xyz fields must be float: " + path);
      }
    } else if (key == "POINTS") {
      tokens >> point_count;
      saw_points = true;
    } else if (key == "DATA") {
      std::string mode;
      tokens >> mode;
      if (mode != "binary") {
        throw std::runtime_error("only binary pcd is supported: " + path);
      }
      cloud.binary = true;
      data_offset = stream.tellg();
      break;
    }
  }

  if (!saw_points || data_offset == std::streampos(0)) {
    throw std::runtime_error("pcd missing POINTS or DATA header: " + path);
  }

  cloud.points.resize(point_count);
  const std::streamsize bytes = static_cast<std::streamsize>(point_count * 12);
  stream.read(reinterpret_cast<char*>(cloud.points.data()), bytes);
  if (stream.gcount() != bytes) {
    throw std::runtime_error("truncated pcd payload: " + path);
  }
  return cloud;
}

void write_pcd_xyz(const std::string& path, const std::vector<Eigen::Vector3f>& points) {
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot write pcd: " + path);
  }
  stream << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z\n"
         << "SIZE 4 4 4\n"
         << "TYPE F F F\n"
         << "COUNT 1 1 1\n"
         << "WIDTH " << points.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << points.size() << "\n"
         << "DATA binary\n";
  stream.write(reinterpret_cast<const char*>(points.data()),
               static_cast<std::streamsize>(points.size() * sizeof(Eigen::Vector3f)));
}

}  // namespace brightdrive
