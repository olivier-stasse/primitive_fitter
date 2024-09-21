#include <fstream>
#include <regex>

#include <nimbro_primitive_fitter/primitive_fitter.h>
#include <nimbro_primitive_fitter/inertia.h>




int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  std::shared_ptr<PrimitiveFitterNode> aPrimitiveFitterNode_shr_ptr =
      std::make_shared<PrimitiveFitterNode>();
  aPrimitiveFitterNode_shr_ptr->setSharedPointer(aPrimitiveFitterNode_shr_ptr);
  aPrimitiveFitterNode_shr_ptr->main();
  rclcpp::spin(aPrimitiveFitterNode_shr_ptr);
  rclcpp::shutdown();  
}
