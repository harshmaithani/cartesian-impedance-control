/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/


#include <reflexxes/ReflexxesAPI.h>
#include <reflexxes/RMLPositionFlags.h>
#include <reflexxes/RMLPositionInputParameters.h>
#include <reflexxes/RMLPositionOutputParameters.h>

// ROS headers
#include <ros/ros.h>

int main( int argc, char** argv )
{
	
	// initialize ROS
	ros::init(argc, argv, "node_test_irml", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;

	
	ReflexxesAPI  *RML_;
	
    RMLPositionInputParameters *IP_;
	RMLPositionOutputParameters *OP_;
	RMLPositionFlags            Flags   ;
	double cycleTime_;
	int resultValue_;
	
	cycleTime_ = 0.002;
        
	RML_ = new ReflexxesAPI(3,cycleTime_);
	IP_ = new RMLPositionInputParameters(3);
	OP_ = new RMLPositionOutputParameters(3);
	
	IP_->CurrentPositionVector->VecData[0] = (double)1.0; 
	IP_->CurrentPositionVector->VecData[1] = (double)1.0;
	IP_->CurrentPositionVector->VecData[2] = (double)1.0;
	
	IP_->CurrentVelocityVector->VecData      [0] =    0.0      ;
    IP_->CurrentVelocityVector->VecData      [1] =   0.0       ;
    IP_->CurrentVelocityVector->VecData      [2] =    0.0       ;

    IP_->CurrentAccelerationVector->VecData  [0] =   0.0      ;
    IP_->CurrentAccelerationVector->VecData  [1] =    0.0      ;
    IP_->CurrentAccelerationVector->VecData  [2] =    0.0      ;

    IP_->MaxVelocityVector->VecData          [0] =    0.05      ;
    IP_->MaxVelocityVector->VecData          [1] =    0.05      ;
    IP_->MaxVelocityVector->VecData          [2] =    0.05     ;

    IP_->MaxAccelerationVector->VecData      [0] =    1.0    ;
    IP_->MaxAccelerationVector->VecData      [1] =    1.0      ;
    IP_->MaxAccelerationVector->VecData      [2] =    1.0      ;

    IP_->MaxJerkVector->VecData              [0] =    0.0001      ;
    IP_->MaxJerkVector->VecData              [1] =    0.0001      ;
    IP_->MaxJerkVector->VecData              [2] =    0.0001      ;

    IP_->TargetPositionVector->VecData       [0] =   2.0      ;
    IP_->TargetPositionVector->VecData       [1] =   2.0      ;
    IP_->TargetPositionVector->VecData       [2] =   2.0      ;

    /*IP_->TargetVelocityVector->VecData       [0] =  0.05       ;
    IP_->TargetVelocityVector->VecData       [1] =  0.05     ;
    IP_->TargetVelocityVector->VecData       [2] =  0.05    ;*/

    IP_->SelectionVector->VecData            [0] =   true        ;
    IP_->SelectionVector->VecData            [1] =   true        ;
    IP_->SelectionVector->VecData            [2] =   true        ;
	
	
	
	for (size_t i=0; i<30; i++)
	{
		ROS_INFO("***************************************** !\n");
	}
		 

	resultValue_ = ReflexxesAPI::RML_WORKING;
	
	ROS_INFO("Start going to position !\n");
	
	while (resultValue_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
	{
					resultValue_ = RML_->RMLPosition(*IP_,OP_, Flags);
					
					if (resultValue_ < 0)
					{
						ROS_INFO("node test irml : ERROR during trajectory generation err n°%d",resultValue_);
						ROS_INFO("x=%f, y=%f, z=%f",OP_->NewPositionVector->VecData[0],OP_->NewPositionVector->VecData[1],OP_->NewPositionVector->VecData[2]);
						break;
					}
					
					ROS_INFO("Pos -> x=%f, y=%f, z=%f",OP_->NewPositionVector->VecData[0],OP_->NewPositionVector->VecData[1],OP_->NewPositionVector->VecData[2]);
					
					ROS_INFO("Vel -> x=%f, y=%f, z=%f",OP_->NewVelocityVector->VecData[0],OP_->NewVelocityVector->VecData[1],OP_->NewVelocityVector->VecData[2]);
					
					ROS_INFO("Acc -> x=%f, y=%f, z=%f",OP_->NewAccelerationVector->VecData[0],OP_->NewAccelerationVector->VecData[1],OP_->NewAccelerationVector->VecData[2]);
					
					/*
					x_acc = OP_->NewVelocity->VecData[0] // x acc
					y_acc = OP_->NewVelocity->VecData[1] // y acc
					z_acc = OP_->NewVelocity->VecData[2] // z acc*/
					
					
					*IP_->CurrentPositionVector      =   *OP_->NewPositionVector      ;
					*IP_->CurrentVelocityVector      =   *OP_->NewVelocityVector      ;
					*IP_->CurrentAccelerationVector  =   *OP_->NewAccelerationVector  ;
	
	}
	
	ROS_INFO("Got Position !!\n");

	delete RML_;
	delete IP_;
	delete OP_;	
	
	ros::spin();
	return 0;
  
}









