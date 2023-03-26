#include <string>
#include <iostream>
#include <sstream>

#include "Alembic/AbcCoreOgawa/All.h"
#include "Alembic/AbcGeom/All.h"
#include "boost/range/adaptor/indexed.hpp"
#include "boost/assign.hpp"


using namespace Alembic;


void print_space(int num)
{
  std::ostringstream oss;
  for (int i = 0; i < num; ++i)
  {
    oss << " ";
  }
  std::cout << oss.str();
}


void print_object(const Alembic::Abc::IObject& object, int& level)
{
  print_space(level);
  std::cout << object.getName() << ' ' << object.getMetaData().serialize() << '\n';

  if (AbcGeom::IXform::matches(object.getHeader()))
  {
    auto xform_obj = AbcGeom::IXform(object);
    if (!xform_obj.valid()) { throw std::runtime_error("Invalid transformation object."); }

    const auto& xform = xform_obj.getSchema();
    if (!xform.valid()) { throw std::runtime_error("Invalid transformation."); }

    for (size_t t = 0; t < xform.getNumSamples(); ++t)
    {
      const auto& xform_sample = xform.getValue(Abc::ISampleSelector(static_cast<int64_t>(t)));
      for (std::size_t i = 0; i < xform_sample.getNumOps(); ++i)
      {
        print_space(level);
        auto xformop = xform_sample.getOp(i);
        std::cout << xformop.getType() << ':';
        for (std::size_t j = 0; j < xformop.getNumChannels(); ++j)
        {
          std::cout << xformop.getChannelValue(i) << ',';
        }
        std::cout << '\n';
      }
    }
  }
  if (AbcGeom::IPolyMesh::matches(object.getHeader()))
  {
    // Get the mesh schema.
    auto mesh_obj = AbcGeom::IPolyMesh(object);

    const auto& mesh = mesh_obj.getSchema();

    // Get geometry data.
    const auto& face_indexes_prop = mesh.getFaceIndicesProperty();

    const auto& face_counts_prop = mesh.getFaceCountsProperty();

    const auto& positions_prop = mesh.getPositionsProperty();

    const auto& normals_param = mesh.getNormalsParam();

    for (size_t t = 0; t < mesh.getNumSamples(); ++t)
    {
      auto iss = Abc::ISampleSelector(static_cast<int64_t>(t));
      auto face_indexes_ptr = face_indexes_prop.getValue(iss);
      auto face_counts_ptr = face_counts_prop.getValue(iss);
      auto positions_ptr = positions_prop.getValue(iss);
      auto normals_ptr = normals_param.getExpandedValue(iss).getVals();

      for (std::size_t i = 0; i != positions_ptr->size(); ++i)
      {
        const auto& p = (*positions_ptr)[i];
        print_space(level);
        std::cout << p.x << ',' << p.y << ',' << p.z << '\n';
      }
    }
  }

  ++level;
  for (std::size_t i = 0; i < object.getNumChildren(); ++i)
  {
    print_object(object.getChild(i), level);
  }
  --level;
}


int main(int argc, char *argv[])
{
  if (argc < 2)
  {
    throw std::runtime_error("Must specify an Alembic file path.");
  }
  // Make Archive.
  auto archive_reader = AbcCoreOgawa::ReadArchive();
  auto reader_ptr = archive_reader(std::string(argv[1]));
  auto archive = Abc::IArchive(reader_ptr);
  if (!archive.valid())
  {
    throw std::runtime_error("Invalid Alembic file.");
  }

  int level = 0;

  print_object(archive.getTop(), level);
}