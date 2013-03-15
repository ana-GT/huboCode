#include "common/CommonTypes.hh"
#include "common/Animation.hh"
#include "common/KeyFrame.hh"
#include "physics/Model.hh"
#include "gazebo.hh"
#include <stdio.h>


namespace gazebo
{
  /**
   * @class AnimatePose
   * @brief Check if the swing is not crazy
   */
  class AnimatePose : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      double count = 300;
      double dt = 0.005; // 200Hz
      double totalTime = 10;
      dt = totalTime / count;
      double t = 0; 

      gazebo::common::PoseAnimationPtr anim( new gazebo::common::PoseAnimation("swing", totalTime, true));

      gazebo::common::PoseKeyFrame *key;

      // Save from the file and play
      char name[80];
      FILE *pFile;
      float x, y, z, yaw;

      pFile = fopen("pos.txt", "r");
      
      for( int i = 0; i < count; ++i ) {
	key = anim->CreateKeyFrame(t);
	fscanf( pFile, "%f %f %f %f \n", &x, &y, &z, &yaw );
	printf(" pos and rot: %f %f %f - %f \n", x, y, z, yaw );
	key->SetTranslation(math::Vector3(x, y, z + 0.1));
	key->SetRotation(math::Quaternion(0, 0, yaw));
	t += dt; 
      }
      fclose( pFile );
      printf("End reading info \n");

      _parent->SetAnimation(anim);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatePose)
}
