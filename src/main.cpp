#include <fstream>
#include <regex>

#include <nimbro_primitive_fitter/primitive_fitter.h>
#include <nimbro_primitive_fitter/inertia.h>

#include <wrap/io_trimesh/import_stl.h>
#include <vcg/complex/algorithms/inertia.h>

#include <rclcpp/rclcpp.hpp>

#include <nimbro_primitive_fitter/primitive_fitter.h>
#include <nimbro_primitive_fitter/inertia.h>


#define NODE_NAME "nimbro_primitive_fitter"
#include <ament_index_cpp/get_package_share_directory.hpp>

class PrimitiveFitterNode : public rclcpp::Node {
 public:
  PrimitiveFitterNode()
      : Node(NODE_NAME) {
    RCLCPP_INFO(this->get_logger(),
                " ********** NimbRo Primitive Fitter started ********** ");

    declare_parameter("urdf_filename", rclcpp::PARAMETER_STRING);
    declare_parameter("output_filename", rclcpp::PARAMETER_STRING);
    declare_parameter("fit_shape", rclcpp::PARAMETER_STRING);
    declare_parameter("use_fitter", rclcpp::PARAMETER_BOOL);
    declare_parameter("use_inertia", rclcpp::PARAMETER_BOOL);
    declare_parameter("mesh_type", rclcpp::PARAMETER_STRING);    
    main();
  };
  ~PrimitiveFitterNode() {};

  std::string getFileExtension(std::string meshfile)
  {
    std::string file_extension("");
    
    // Get file extension
    std::regex file_regex("([^\\\\.]+)(\\.)([^\\\\.]+)");
    std::smatch r_match;
    
    if (std::regex_search(meshfile, r_match, file_regex))
    {
      file_extension = r_match[3];
      RCLCPP_DEBUG_STREAM(get_logger(), "Detected file extension ." << r_match[3] );
    }
    else
    {
      RCLCPP_INFO(get_logger(), "NO FILE EXTENSION FOUND!" );
    }
    
    return file_extension;
  }

  void update_inertia(std::string in_filename, std::string out_filename,
                      bool visual)
  {
    Urdf urdf(in_filename.c_str(), false,std::shared_ptr<rclcpp::Node>(this));
    
    // Return if the urdf was not loaded
    if ( !urdf.loaded() )
    {
      RCLCPP_ERROR(get_logger(), "URDF could not be loaded." );
      return;
    }

    bool hasMoreMesh;

    if (visual)
    {
      hasMoreMesh = urdf.hasMoreVisualMesh();
    }
    else
    {
      hasMoreMesh = urdf.hasMoreMesh();
    }

    while (hasMoreMesh)
    {
      std::string meshfile;

      if (visual)
      {
        meshfile = urdf.getNextVisualMesh();
        hasMoreMesh = urdf.hasMoreVisualMesh();
      }
      else
      {
        meshfile = urdf.getNextMesh();
        hasMoreMesh = urdf.hasMoreMesh();
      }

      std::string file_extension = getFileExtension(meshfile);

      if (file_extension == "stl" || file_extension == "STL")
      {
        RCLCPP_DEBUG(get_logger(), "Handling STL file ..." );
        RCLCPP_DEBUG_STREAM(get_logger(), "path: " << meshfile );

        // get global meshfile path
        url::Url filepath(meshfile);
        std::string pkgName(filepath.getPackageName());
        // may throw ament_index_cpp::PackageNotFoundError exception
        std::string pkgPath =
            ament_index_cpp::get_package_share_directory(pkgName);
        
        std::string path(filepath.getRelativePath());
        std::string global_meshfile((pkgPath + path).c_str());
        RCLCPP_INFO_STREAM(get_logger(), "global path: " << global_meshfile );

        // open STL file
        MyMesh mesh;
        int mask = 0;

        if(vcg::tri::io::ImporterSTL<MyMesh>::Open(mesh, global_meshfile.c_str(), mask))
        {
          RCLCPP_INFO_STREAM(get_logger(), "Error reading file " << global_meshfile );
          return;
        }

        // read mass from file
        float mass = urdf.getMass(visual);

        // compute inertia and center of mass
        vcg::Matrix33f Inertia;
        vcg::tri::Inertia<MyMesh> I(mesh);
        I.InertiaTensor(Inertia);
        vcg::Point3f CenterOfMass = I.CenterOfMass();

        // scale inertia according to mass
        float volume = I.Mass();
        Inertia /= volume;
        Inertia *= mass;

        // set new values
        urdf.setCenterOfMass(CenterOfMass, visual);
        urdf.setInertia(Inertia, visual);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "UNKNOWN FILE EXTENSION! (only .stl supported)" );
        return;
      }
    }

    if (urdf.save(out_filename.c_str()))
    {
      RCLCPP_INFO(get_logger(), "Inertia updated successfully!" );
    }

    return;
  }

  void main(void) {
    RCLCPP_INFO(this->get_logger(), "step 1 " );
    std::string in_filename;
    try {
      in_filename =
          this->get_parameter("urdf_filename")
              .as_string();
    } catch (...) {
      std::cerr << "Unable to find parameter urdf_filename" << std::endl;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "step 2 ");
    std::string out_filename("");
    out_filename = this->get_parameter("output_filename").as_string();

    RCLCPP_INFO(this->get_logger(), "step 3 " );
    if(out_filename.empty())
    {
      size_t lastindex = in_filename.find_last_of(".");
      out_filename = in_filename.substr(0, lastindex) + "_optimized.urdf.xacro";
    }
    
    RCLCPP_INFO(this->get_logger(), "step 4 " );    
    bool fitter = true;
    fitter = this->get_parameter("use_fitter").as_bool();
    
    bool inertia = false;	
    inertia = this->get_parameter("use_inertia").as_bool();

    RCLCPP_INFO(this->get_logger(), "step 5 " );    
    std::string capsule_filename("");
    
    if (inertia)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Inertia calculation started.");
      
      std::string mesh_type = "visual";
      bool visual = true;
      mesh_type = this->get_parameter("mesh_type").as_string();
      
      if (mesh_type == "visual")
      {
        visual = true;
        RCLCPP_INFO(this->get_logger(),"Using visual meshes.");
      }
      else if (mesh_type == "collision")
      {
        visual = false;
        RCLCPP_INFO(this->get_logger(),
                    "Using collision meshes.");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(),
                    "PLEASE SET ROSPARAM /nimbro_primitive_fitter/mesh_type"
                    " TO EITHER visual OR collsion -> Using 'visual'.");
        visual = true;
      }
		
      update_inertia(in_filename, out_filename, visual);
      in_filename = out_filename;
    }
    
    if (fitter)
    {
      std::string shape = "box";
      shape = this->get_parameter("/" + std::string(NODE_NAME) + "/fit_shape").as_string();
      VersatileFitter::Shape vf_shape;
      
      if (shape == "box" || shape == "Box")
      {
        RCLCPP_INFO(this->get_logger(),
                    "Will try to fit URDF to Box.");
        vf_shape = VersatileFitter::Shape::Box;
      }
      else if (shape == "capsule" || shape == "Capsule")
      {
        RCLCPP_INFO(this->get_logger(),
                    "Will try to fit URDF to Capsule.");
        vf_shape = VersatileFitter::Shape::Capsule;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(),
                     "PLEASE SET ROSPARAM /nimbro_primitive_fitter/fit_shape TO EITHER box OR capsule");
        RCLCPP_INFO(this->get_logger(),"Will try to fit URDF to Box.");
        vf_shape = VersatileFitter::Shape::Box;
      }
      
      VersatileFitter vf(std::shared_ptr<rclcpp::Node>(this));
      vf.fit(in_filename, out_filename, vf_shape, shape);
    }
  };
    
};

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrimitiveFitterNode>());
  rclcpp::shutdown();  
}
