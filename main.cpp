#include <string>
#include <stdexcept>
#include <iostream>
#include <sstream>

#include "Alembic/AbcCoreOgawa/All.h"
#include "Alembic/AbcGeom/All.h"
#include "boost/range/adaptor/indexed.hpp"
#include "boost/assign.hpp"

using namespace Alembic;

void print_space(int num) {
  std::ostringstream oss;
  for (int i = 0; i < num; ++i) {
    oss << " ";
  }
  std::cout << oss.str();
}

void print_object(const Alembic::Abc::IObject& object, int& level) {
  print_space(level);
  std::cout << object.getName() << ' ' << object.getMetaData().serialize()
            << '\n';

  if (AbcGeom::IXform::matches(object.getHeader())) {
    auto xform_obj = AbcGeom::IXform(object);
    if (!xform_obj.valid()) {
      throw std::runtime_error("Invalid transformation object.");
    }

    const auto& xform = xform_obj.getSchema();
    if (!xform.valid()) {
      throw std::runtime_error("Invalid transformation.");
    }

    for (size_t t = 0; t < xform.getNumSamples(); ++t) {
      print_space(level);
      std::cout << '[' << t << ']' << '\n';
      const auto& xform_sample =
          xform.getValue(Abc::ISampleSelector(static_cast<int64_t>(t)));
      for (std::size_t i = 0; i < xform_sample.getNumOps(); ++i) {
        auto xformop = xform_sample.getOp(i);
        print_space(level);
        std::cout << xformop.getType() << ':';
        for (std::size_t j = 0; j < xformop.getNumChannels(); ++j) {
          std::cout << xformop.getChannelValue(j) << ',';
        }
        std::cout << '\n';
      }
    }

    auto time_sampling = *(xform.getTimeSampling());
    print_space(level);
    std::cout << "time samplings\n";
    std::size_t i = 0;
    const auto& stored_times = time_sampling.getStoredTimes();
    for (auto t : stored_times) {
      print_space(level);
      std::cout << i << " : " << t << '\n';
      ++i;
    }

    auto time_sampling_type = time_sampling.getTimeSamplingType();
    print_space(level);
    std::cout << "time sampling type\n";
    if (time_sampling_type.isUniform()) {
      print_space(level);
      std::cout << "uniform\n";
    } else if (time_sampling_type.isCyclic()) {
      print_space(level);
      std::cout << "cyclic\n";
    } else if (time_sampling_type.isAcyclic()) {
      print_space(level);
      std::cout << "acyclic\n";
    } else {
      print_space(level);
      std::cout << "unknown\n";
    }
    std::cout << "num samples per cycle: "
              << time_sampling_type.getNumSamplesPerCycle() << '\n';
    std::cout << "time per cycle: " << time_sampling_type.getTimePerCycle()
              << '\n';
  }
  if (AbcGeom::IPolyMesh::matches(object.getHeader())) {
    // Get the mesh schema.
    auto mesh_obj = AbcGeom::IPolyMesh(object);

    const auto& mesh = mesh_obj.getSchema();

    // check if the topology of the transforming multi-polygon is unchanged.
    switch (mesh.getTopologyVariance()) {
      case Alembic::AbcGeom::MeshTopologyVariance::kConstantTopology:
        std::cout << "constant topology\n";
        break;
      case Alembic::AbcGeom::MeshTopologyVariance::kHomogeneousTopology:
        std::cout << "homogeneous topology\n";
        break;
      case Alembic::AbcGeom::MeshTopologyVariance::kHeterogeneousTopology:
        std::cout << "heterogeneous topology\n";
        break;
      default:
        std::cout << "unknown topology\n";
        break;
    }

    // Get geometry data.
    const auto& face_indexes_prop = mesh.getFaceIndicesProperty();

    const auto& face_counts_prop = mesh.getFaceCountsProperty();

    const auto& positions_prop = mesh.getPositionsProperty();

    const auto& normals_param = mesh.getNormalsParam();

    print_space(level);
    std::cout << "normal scope: " << normals_param.getScope() << "\n";

    for (size_t t = 0; t < mesh.getNumSamples(); ++t) {
      auto iss = Abc::ISampleSelector(static_cast<int64_t>(t));
      auto face_indexes_ptr = face_indexes_prop.getValue(iss);
      auto face_counts_ptr = face_counts_prop.getValue(iss);
      auto positions_ptr = positions_prop.getValue(iss);
      auto normals_ptr = normals_param.getExpandedValue(iss).getVals();

      print_space(level);
      std::cout << "points:\n";
      for (std::size_t i = 0; i != positions_ptr->size(); ++i) {
        const auto& p = (*positions_ptr)[i];
        print_space(level);
        std::cout << p.x << ',' << p.y << ',' << p.z << '\n';
      }

      print_space(level);
      std::cout << "normals:\n";
      for (std::size_t i = 0; i != normals_ptr->size(); ++i) {
        const auto& n = (*normals_ptr)[i];
        print_space(level);
        std::cout << n.x << ',' << n.y << ',' << n.z << '\n';
      }

      print_space(level);
      std::cout << "faces:\n";
      std::size_t global_face_index = 0;
      for (std::size_t i = 0; i != face_counts_ptr->size(); ++i) {
        print_space(level);
        for (std::size_t j = 0;
             j != static_cast<std::size_t>((*face_counts_ptr)[i]); ++j) {
          std::cout << (*face_indexes_ptr)[global_face_index + j] << ',';
        }
        std::cout << '\n';
        global_face_index += (*face_counts_ptr)[i];
      }
    }

    std::cout << "velocities:\n";
    const auto& velocities_prop = mesh.getVelocitiesProperty();
    if (velocities_prop.valid()) {
      for (size_t t = 0; t < mesh.getNumSamples(); ++t) {
        auto iss = Abc::ISampleSelector(static_cast<int64_t>(t));
        auto velocities_ptr = velocities_prop.getValue(iss);
        for (std::size_t i = 0; i != velocities_ptr->size(); ++i) {
          const auto& v = (*velocities_ptr)[i];
          print_space(level);
          std::cout << v.x << ',' << v.y << ',' << v.z << '\n';
        }
      }
    }

    auto time_sampling = *(mesh.getTimeSampling());
    print_space(level);
    std::cout << "time samplings\n";
    std::size_t i = 0;
    const auto& stored_times = time_sampling.getStoredTimes();
    for (auto t : stored_times) {
      print_space(level);
      std::cout << i << " : " << t << '\n';
      ++i;
    }

    auto time_sampling_type = time_sampling.getTimeSamplingType();
    print_space(level);
    std::cout << "time sampling type\n";
    if (time_sampling_type.isUniform()) {
      print_space(level);
      std::cout << "uniform\n";
    } else if (time_sampling_type.isCyclic()) {
      print_space(level);
      std::cout << "cyclic\n";
    } else if (time_sampling_type.isAcyclic()) {
      print_space(level);
      std::cout << "acyclic\n";
    } else {
      print_space(level);
      std::cout << "unknown\n";
    }
    std::cout << "num samples per cycle: "
              << time_sampling_type.getNumSamplesPerCycle() << '\n';
    std::cout << "time per cycle: " << time_sampling_type.getTimePerCycle()
              << '\n';
  }

  if (AbcGeom::IPoints::matches(object.getHeader())) {
    // Get the mesh schema.
    auto points_obj = AbcGeom::IPoints(object);

    const auto& points = points_obj.getSchema();

    for (size_t t = 0; t < points.getNumSamples(); ++t) {
      auto iss = Abc::ISampleSelector(static_cast<int64_t>(t));
      auto sample = points.getValue(iss);
      auto p3f_ptr = sample.getPositions();
      auto id_ptr = sample.getIds();
      auto velocity_ptr = sample.getVelocities();

      print_space(level);
      std::cout << "points:\n";
      for (std::size_t i = 0; i != p3f_ptr->size(); ++i) {
        const auto& p = (*p3f_ptr)[i];
        print_space(level);
        std::cout << p.x << ',' << p.y << ',' << p.z << '\n';
      }

      print_space(level);
      std::cout << "ids:\n";
      for (std::size_t i = 0; i != id_ptr->size(); ++i) {
        const auto& id = (*id_ptr)[i];
        print_space(level);
        std::cout << id << '\n';
      }

      print_space(level);
      std::cout << "velocities:\n";
      for (std::size_t i = 0; i != velocity_ptr->size(); ++i) {
        const auto& p = (*velocity_ptr)[i];
        print_space(level);
        std::cout << p.x << ',' << p.y << ',' << p.z << '\n';
      }
    }
  }

  ++level;
  for (std::size_t i = 0; i < object.getNumChildren(); ++i) {
    print_object(object.getChild(i), level);
  }
  --level;
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    throw std::runtime_error("Must specify an Alembic file path.");
  }
  // Make Archive.
  auto archive_reader = AbcCoreOgawa::ReadArchive();
  auto reader_ptr = archive_reader(std::string(argv[1]));
  auto archive = Abc::IArchive(reader_ptr);
  if (!archive.valid()) {
    throw std::runtime_error("Invalid Alembic file.");
  }

  int level = 0;

  print_object(archive.getTop(), level);
}