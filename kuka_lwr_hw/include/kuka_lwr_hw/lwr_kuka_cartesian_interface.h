/*
 * Laurent LEQUIEVRE
 * Ingénieur CNRS
 * laurent.lequievre@uca.fr
 * Institut Pascal UMR6602
 * 
*/

#ifndef HARDWARE_INTERFACE_LWR_KUKA_CARTESIAN_INTERFACE_H
#define HARDWARE_INTERFACE_LWR_KUKA_CARTESIAN_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{
	
	/** A KUKA cartesian pose state handle class. */
	class KukaCartesianPoseStateHandle
	{
		public:
			KukaCartesianPoseStateHandle(): name_(), rxx_(0), rxy_(0), rxz_(0), tx_(0), ryx_(0), ryy_(0), ryz_(0), ty_(0), rzx_(0), rzy_(0), rzz_(0), tz_(0) {}
			
			KukaCartesianPoseStateHandle(const std::string& name, const double* rxx, const double* rxy, const double* rxz, const double* tx, const double* ryx, const double* ryy, const double* ryz, const double* ty, const double* rzx, const double* rzy, const double* rzz, const double* tz)
			: name_(name), rxx_(rxx), rxy_(rxy), rxz_(rxz), ryx_(ryx), ryy_(ryy), ryz_(ryz), rzx_(rzx), rzy_(rzy), rzz_(rzz), tx_(tx), ty_(ty), tz_(tz)
		    {
				if (!rxx)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. RXX data pointer is null.");
				}
				if (!rxy)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. RXY data pointer is null.");
				}
				if (!rxz)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. RXZ data pointer is null.");
				}
				
				if (!ryx)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. RYX data pointer is null.");
				}
				if (!ryy)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. RYY data pointer is null.");
				}
				if (!ryz)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. RYZ data pointer is null.");
				}
				
				if (!rzx)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. RZX data pointer is null.");
				}
				if (!rzy)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. RZY data pointer is null.");
				}
				if (!rzz)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. RZZ data pointer is null.");
				}
				
				if (!tx)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. TX data pointer is null.");
				}
				if (!ty)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. TY data pointer is null.");
				}
				if (!tz)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Pose State handle '" + name + "'. TZ data pointer is null.");
				}
				
		    }
			
			std::string getName()     const {return name_;}
			
			double getRXX()      	  const {assert(rxx_); return *rxx_;}
			double getRXY()      	  const {assert(rxy_); return *rxy_;}
			double getRXZ()      	  const {assert(rxz_); return *rxz_;}
			
			double getRYX()      	  const {assert(ryx_); return *ryx_;}
			double getRYY()      	  const {assert(ryy_); return *ryy_;}
			double getRYZ()      	  const {assert(ryz_); return *ryz_;}
			
			double getRZX()      	  const {assert(rzx_); return *rzx_;}
			double getRZY()      	  const {assert(rzy_); return *rzy_;}
			double getRZZ()      	  const {assert(rzz_); return *rzz_;}
			
			double getTX()      	  const {assert(tx_); return *tx_;}
			double getTY()      	  const {assert(ty_); return *ty_;}
			double getTZ()      	  const {assert(tz_); return *tz_;}
				
		private:
		  std::string name_;
		  const double* rxx_;
		  const double* rxy_;
		  const double* rxz_;
		  const double* tx_;
		  const double* ryx_;
		  const double* ryy_;
		  const double* ryz_;
		  const double* ty_;
		  const double* rzx_;
		  const double* rzy_;
		  const double* rzz_;
		  const double* tz_; 
	};
	
	/** A KUKA cartesian wrench state handle class. */
	class KukaCartesianWrenchStateHandle
	{
		public:
		  KukaCartesianWrenchStateHandle(): name_(), x_(0), y_(0), z_(0), a_(0), b_(0), c_(0) {}

		  /**
		   * \param name The name of the wrench handle
		   */
		  KukaCartesianWrenchStateHandle(const std::string& name, const double* x, const double* y, const double* z, const double* a, const double* b, const double* c)
			: name_(name), x_(x), y_(y), z_(z), a_(a), b_(b), c_(c)
		  {
				if (!x)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Wrench State handle '" + name + "'. X data pointer is null.");
				}
				if (!y)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Wrench State handle '" + name + "'. Y data pointer is null.");
				}
				if (!z)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Wrench State handle '" + name + "'. Z data pointer is null.");
				}
				if (!a)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Wrench State handle '" + name + "'. A data pointer is null.");
				}
				if (!b)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Wrench State handle '" + name + "'. B data pointer is null.");
				}
				if (!c)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Wrench State handle '" + name + "'. C data pointer is null.");
				}
		  }

		  std::string getName()     const {return name_;}
		  double getX()      		const {assert(x_); return *x_;}
		  double getY()      		const {assert(y_); return *y_;}
		  double getZ()      		const {assert(z_); return *z_;}
		  double getA()      		const {assert(a_); return *a_;}
		  double getB()      		const {assert(b_); return *b_;}
		  double getC()      		const {assert(c_); return *c_;}

		private:
		  std::string name_;
		  const double* x_;
		  const double* y_;
		  const double* z_;
		  const double* a_;
		  const double* b_;
		  const double* c_;
	};
	
	/** A KUKA cartesian stiffness state handle class. */
	class KukaCartesianStiffnessStateHandle
	{
		public:
		  KukaCartesianStiffnessStateHandle(): name_(), x_(0), y_(0), z_(0), a_(0), b_(0), c_(0) {}

		  /**
		   * \param name The name of the stiffness handle
		   */
		  KukaCartesianStiffnessStateHandle(const std::string& name, const double* x, const double* y, const double* z, const double* a, const double* b, const double* c)
			: name_(name), x_(x), y_(y), z_(z), a_(a), b_(b), c_(c)
		  {
				if (!x)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. X data pointer is null.");
				}
				if (!y)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. Y data pointer is null.");
				}
				if (!z)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. Z data pointer is null.");
				}
				if (!a)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. A data pointer is null.");
				}
				if (!b)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. B data pointer is null.");
				}
				if (!c)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. C data pointer is null.");
				}
		  }

		  std::string getName()     const {return name_;}
		  double getX()      		const {assert(x_); return *x_;}
		  double getY()      		const {assert(y_); return *y_;}
		  double getZ()      		const {assert(z_); return *z_;}
		  double getA()      		const {assert(a_); return *a_;}
		  double getB()      		const {assert(b_); return *b_;}
		  double getC()      		const {assert(c_); return *c_;}

		private:
		  std::string name_;
		  const double* x_;
		  const double* y_;
		  const double* z_;
		  const double* a_;
		  const double* b_;
		  const double* c_;
	};
	
	
	/** A KUKA cartesian damping state handle class. */
	class KukaCartesianDampingStateHandle
	{
		public:
		  KukaCartesianDampingStateHandle(): name_(), x_(0), y_(0), z_(0), a_(0), b_(0), c_(0) {}

		  /**
		   * \param name The name of the stiffness handle
		   */
		  KukaCartesianDampingStateHandle(const std::string& name, const double* x, const double* y, const double* z, const double* a, const double* b, const double* c)
			: name_(name), x_(x), y_(y), z_(z), a_(a), b_(b), c_(c)
		  {
				if (!x)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. X data pointer is null.");
				}
				if (!y)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. Y data pointer is null.");
				}
				if (!z)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. Z data pointer is null.");
				}
				if (!a)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. A data pointer is null.");
				}
				if (!b)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. B data pointer is null.");
				}
				if (!c)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. C data pointer is null.");
				}
		  }

		  std::string getName()     const {return name_;}
		  double getX()      		const {assert(x_); return *x_;}
		  double getY()      		const {assert(y_); return *y_;}
		  double getZ()      		const {assert(z_); return *z_;}
		  double getA()      		const {assert(a_); return *a_;}
		  double getB()      		const {assert(b_); return *b_;}
		  double getC()      		const {assert(c_); return *c_;}

		private:
		  std::string name_;
		  const double* x_;
		  const double* y_;
		  const double* z_;
		  const double* a_;
		  const double* b_;
		  const double* c_;
	};
	
	/** \brief A handle used to read and command a KUKA cartesian pose. */
	class KUKACartesianPoseHandle : public KukaCartesianPoseStateHandle
	{
		public:
			KUKACartesianPoseHandle() : KukaCartesianPoseStateHandle(), 
			rxx_cmd_(0), rxy_cmd_(0), rxz_cmd_(0), tx_cmd_(0), 
			ryx_cmd_(0), ryy_cmd_(0), ryz_cmd_(0), ty_cmd_(0), 
			rzx_cmd_(0), rzy_cmd_(0), rzz_cmd_(0), tz_cmd_(0) 
			{}
			
			/**
			* \param js This pose's state handle
			* \param cmd A pointer to the storage for this joint's output command
			*/
			KUKACartesianPoseHandle(const KukaCartesianPoseStateHandle& js, double* rxx_cmd, double* rxy_cmd, double* rxz_cmd, double* tx_cmd, double* ryx_cmd, double* ryy_cmd, double* ryz_cmd, double* ty_cmd, double* rzx_cmd, double* rzy_cmd, double* rzz_cmd, double* tz_cmd)
			: KukaCartesianPoseStateHandle(js), 
				rxx_cmd_(rxx_cmd), rxy_cmd_(rxy_cmd), rxz_cmd_(rxz_cmd), tx_cmd_(tx_cmd),
				ryx_cmd_(ryx_cmd), ryy_cmd_(ryy_cmd), ryz_cmd_(ryz_cmd), ty_cmd_(ty_cmd),
				rzx_cmd_(rzx_cmd), rzy_cmd_(rzy_cmd), rzz_cmd_(rzz_cmd), tz_cmd_(tz_cmd)
			  {
				if (!rxx_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] RXX pointer is null.");
				}
				if (!rxy_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] RXY pointer is null.");
				}
				if (!rxz_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] RXZ pointer is null.");
				}
				if (!tx_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] TX pointer is null.");
				}
				
				if (!ryx_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] RYX pointer is null.");
				}
				if (!ryy_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] RYY pointer is null.");
				}
				if (!ryz_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] RYZ pointer is null.");
				}
				if (!ty_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] TY pointer is null.");
				}
				
				if (!rzx_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] RZX pointer is null.");
				}
				if (!rzy_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] RZY pointer is null.");
				}
				if (!rzz_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] RZZ pointer is null.");
				}
				if (!tz_cmd_)
				{
				  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Pose] TZ pointer is null.");
				}
	
			  }
			  
		  void setCommandRXX(double command)    {assert(rxx_cmd_); *rxx_cmd_ = command;}
		  void setCommandRXY(double command)    {assert(rxy_cmd_); *rxy_cmd_ = command;}
		  void setCommandRXZ(double command)    {assert(rxz_cmd_); *rxz_cmd_ = command;}
		  void setCommandTX(double command)     {assert(tx_cmd_);  *tx_cmd_ = command;}
		  
		  void setCommandRYX(double command)    {assert(ryx_cmd_); *ryx_cmd_ = command;}
		  void setCommandRYY(double command)    {assert(ryy_cmd_); *ryy_cmd_ = command;}
		  void setCommandRYZ(double command)    {assert(ryz_cmd_); *ryz_cmd_ = command;}
		  void setCommandTY(double command)     {assert(ty_cmd_);  *ty_cmd_ = command;}
		  
		  void setCommandRZX(double command)    {assert(rzx_cmd_); *rzx_cmd_ = command;}
		  void setCommandRZY(double command)    {assert(rzy_cmd_); *rzy_cmd_ = command;}
		  void setCommandRZZ(double command)    {assert(rzz_cmd_); *rzz_cmd_ = command;}
		  void setCommandTZ(double command)     {assert(tz_cmd_);  *tz_cmd_ = command;}
		  
		  double getCommandRXX()    const {assert(rxx_cmd_); return *rxx_cmd_;}
		  double getCommandRXY()    const {assert(rxy_cmd_); return *rxy_cmd_;}
		  double getCommandRXZ()    const {assert(rxz_cmd_); return *rxz_cmd_;}
		  double getCommandTX()    const {assert(tx_cmd_); return *tx_cmd_;}
		  
		  double getCommandRYX()    const {assert(ryx_cmd_); return *ryx_cmd_;}
		  double getCommandRYY()    const {assert(ryy_cmd_); return *ryy_cmd_;}
		  double getCommandRYZ()    const {assert(ryz_cmd_); return *ryz_cmd_;}
		  double getCommandTY()    const {assert(ty_cmd_); return *ty_cmd_;}
		  
		  double getCommandRZX()    const {assert(rzx_cmd_); return *rzx_cmd_;}
		  double getCommandRZY()    const {assert(rzy_cmd_); return *rzy_cmd_;}
		  double getCommandRZZ()    const {assert(rzz_cmd_); return *rzz_cmd_;}
		  double getCommandTZ()    const {assert(tz_cmd_); return *tz_cmd_;}
		  
	  private:

		  double* rxx_cmd_;
		  double* rxy_cmd_;
		  double* rxz_cmd_;
		  double* tx_cmd_;
		  
		  double* ryx_cmd_;
		  double* ryy_cmd_;
		  double* ryz_cmd_;
		  double* ty_cmd_;
		  
		  double* rzx_cmd_;
		  double* rzy_cmd_;
		  double* rzz_cmd_;
		  double* tz_cmd_;

	};
	
	
	/** \brief A handle used to read and command a KUKA cartesian wrench. */
	class KUKACartesianWrenchHandle : public KukaCartesianWrenchStateHandle
	{
	public:
	  KUKACartesianWrenchHandle() : KukaCartesianWrenchStateHandle(), x_cmd_(0), y_cmd_(0), z_cmd_(0), a_cmd_(0), b_cmd_(0), c_cmd_(0) {}

	  /**
	   * \param js This wrench's state handle
	   * \param cmd A pointer to the storage for this wrench's output command
	   */
	  KUKACartesianWrenchHandle(const KukaCartesianWrenchStateHandle& js, double* x_cmd, double* y_cmd, double* z_cmd, double* a_cmd, double* b_cmd, double* c_cmd)
		: KukaCartesianWrenchStateHandle(js), x_cmd_(x_cmd), y_cmd_(y_cmd), z_cmd_(z_cmd), a_cmd_(a_cmd), b_cmd_(b_cmd), c_cmd_(c_cmd)
	  {
		if (!x_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Wrench] X pointer is null.");
		}

		if (!y_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Wrench] Y pointer is null.");
		}

		if (!z_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Wrench] Z pointer is null.");
		}

		if (!a_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Wrench] A pointer is null.");
		}
		
		if (!b_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Wrench] B pointer is null.");
		}
		
		if (!c_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Wrench] C pointer is null.");
		}
	  }

	  void setCommandX(double command)    {assert(x_cmd_); *x_cmd_ = command;}
	  void setCommandY(double command)    {assert(y_cmd_); *y_cmd_ = command;}
	  void setCommandZ(double command)    {assert(z_cmd_); *z_cmd_ = command;}
	  void setCommandA(double command)    {assert(a_cmd_); *a_cmd_ = command;}
	  void setCommandB(double command)    {assert(b_cmd_); *b_cmd_ = command;}
	  void setCommandC(double command)    {assert(c_cmd_); *c_cmd_ = command;}


	  double getCommandX()    const {assert(x_cmd_); return *x_cmd_;}
	  double getCommandY()    const {assert(y_cmd_); return *y_cmd_;}
	  double getCommandZ()    const {assert(z_cmd_); return *z_cmd_;}
	  double getCommandA()    const {assert(a_cmd_); return *a_cmd_;}
	  double getCommandB()    const {assert(b_cmd_); return *b_cmd_;}
	  double getCommandC()    const {assert(c_cmd_); return *c_cmd_;}

	private:

	  double* x_cmd_;
	  double* y_cmd_;
	  double* z_cmd_;
	  double* a_cmd_;
	  double* b_cmd_;
	  double* c_cmd_;

	};
	
	/** \brief A handle used to read and command a KUKA cartesian stiffness. */
	class KUKACartesianStiffnessHandle : public KukaCartesianStiffnessStateHandle
	{
	public:
	  KUKACartesianStiffnessHandle() : KukaCartesianStiffnessStateHandle(), x_cmd_(0), y_cmd_(0), z_cmd_(0), a_cmd_(0), b_cmd_(0), c_cmd_(0) {}

	  /**
	   * \param js This joint's state handle
	   * \param cmd A pointer to the storage for this joint's output command
	   */
	  KUKACartesianStiffnessHandle(const KukaCartesianStiffnessStateHandle& js, double* x_cmd, double* y_cmd, double* z_cmd, double* a_cmd, double* b_cmd, double* c_cmd)
		: KukaCartesianStiffnessStateHandle(js), x_cmd_(x_cmd), y_cmd_(y_cmd), z_cmd_(z_cmd), a_cmd_(a_cmd), b_cmd_(b_cmd), c_cmd_(c_cmd)
	  {
		if (!x_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] X pointer is null.");
		}

		if (!y_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] Y pointer is null.");
		}

		if (!z_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] Z pointer is null.");
		}

		if (!a_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] A pointer is null.");
		}
		
		if (!b_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] B pointer is null.");
		}
		
		if (!c_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] C pointer is null.");
		}
	  }

	  void setCommandX(double command)    {assert(x_cmd_); *x_cmd_ = command;}
	  void setCommandY(double command)    {assert(y_cmd_); *y_cmd_ = command;}
	  void setCommandZ(double command)    {assert(z_cmd_); *z_cmd_ = command;}
	  void setCommandA(double command)    {assert(a_cmd_); *a_cmd_ = command;}
	  void setCommandB(double command)    {assert(b_cmd_); *b_cmd_ = command;}
	  void setCommandC(double command)    {assert(c_cmd_); *c_cmd_ = command;}


	  double getCommandX()    const {assert(x_cmd_); return *x_cmd_;}
	  double getCommandY()    const {assert(y_cmd_); return *y_cmd_;}
	  double getCommandZ()    const {assert(z_cmd_); return *z_cmd_;}
	  double getCommandA()    const {assert(a_cmd_); return *a_cmd_;}
	  double getCommandB()    const {assert(b_cmd_); return *b_cmd_;}
	  double getCommandC()    const {assert(c_cmd_); return *c_cmd_;}

	private:

	  double* x_cmd_;
	  double* y_cmd_;
	  double* z_cmd_;
	  double* a_cmd_;
	  double* b_cmd_;
	  double* c_cmd_;

	};
	
	/** \brief A handle used to read and command a KUKA cartesian damping. */
	class KUKACartesianDampingHandle : public KukaCartesianDampingStateHandle
	{
	public:
	  KUKACartesianDampingHandle() : KukaCartesianDampingStateHandle(), x_cmd_(0), y_cmd_(0), z_cmd_(0), a_cmd_(0), b_cmd_(0), c_cmd_(0) {}

	  /**
	   * \param js This joint's state handle
	   * \param cmd A pointer to the storage for this joint's output command
	   */
	  KUKACartesianDampingHandle(const KukaCartesianDampingStateHandle& js, double* x_cmd, double* y_cmd, double* z_cmd, double* a_cmd, double* b_cmd, double* c_cmd)
		: KukaCartesianDampingStateHandle(js), x_cmd_(x_cmd), y_cmd_(y_cmd), z_cmd_(z_cmd), a_cmd_(a_cmd), b_cmd_(b_cmd), c_cmd_(c_cmd)
	  {
		if (!x_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] X pointer is null.");
		}

		if (!y_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] Y pointer is null.");
		}

		if (!z_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] Z pointer is null.");
		}

		if (!a_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] A pointer is null.");
		}
		
		if (!b_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] B pointer is null.");
		}
		
		if (!c_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] C pointer is null.");
		}
	  }

	  void setCommandX(double command)    {assert(x_cmd_); *x_cmd_ = command;}
	  void setCommandY(double command)    {assert(y_cmd_); *y_cmd_ = command;}
	  void setCommandZ(double command)    {assert(z_cmd_); *z_cmd_ = command;}
	  void setCommandA(double command)    {assert(a_cmd_); *a_cmd_ = command;}
	  void setCommandB(double command)    {assert(b_cmd_); *b_cmd_ = command;}
	  void setCommandC(double command)    {assert(c_cmd_); *c_cmd_ = command;}


	  double getCommandX()    const {assert(x_cmd_); return *x_cmd_;}
	  double getCommandY()    const {assert(y_cmd_); return *y_cmd_;}
	  double getCommandZ()    const {assert(z_cmd_); return *z_cmd_;}
	  double getCommandA()    const {assert(a_cmd_); return *a_cmd_;}
	  double getCommandB()    const {assert(b_cmd_); return *b_cmd_;}
	  double getCommandC()    const {assert(c_cmd_); return *c_cmd_;}

	private:

	  double* x_cmd_;
	  double* y_cmd_;
	  double* z_cmd_;
	  double* a_cmd_;
	  double* b_cmd_;
	  double* c_cmd_;

	};

	class KukaCartesianStiffnessStateInterface : public HardwareResourceManager<KukaCartesianStiffnessStateHandle> {};

	class KUKACartesianStiffnessCommandInterface : public HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources> {};

	class KUKACartesianStiffnessInterface : public KUKACartesianStiffnessCommandInterface {};
	
	
	class KukaCartesianDampingStateInterface : public HardwareResourceManager<KukaCartesianDampingStateHandle> {};

	class KUKACartesianDampingCommandInterface : public HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources> {};

	class KUKACartesianDampingInterface : public KUKACartesianDampingCommandInterface {};
	
	
	class KukaCartesianPoseStateInterface : public HardwareResourceManager<KukaCartesianPoseStateHandle> {};

	class KUKACartesianPoseCommandInterface : public HardwareResourceManager<KUKACartesianPoseHandle, ClaimResources> {};

	class KUKACartesianPoseInterface : public KUKACartesianPoseCommandInterface {};
	
	
	class KukaCartesianWrenchStateInterface : public HardwareResourceManager<KukaCartesianWrenchStateHandle> {};

	class KUKACartesianWrenchCommandInterface : public HardwareResourceManager<KUKACartesianWrenchHandle, ClaimResources> {};

	class KUKACartesianWrenchInterface : public KUKACartesianWrenchCommandInterface {};
	
	

	 /** \brief Hardware interface to support commanding KUKA LWR 4+ cartesian.
	 *
	 *
	 * \note Getting a cartesian variable handle through the getHandle() method \e will claim that resource.
	 *
	 */
	class KUKACartesianCommandInterface : 	public HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>, 
											public HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>,
											public HardwareResourceManager<KUKACartesianPoseHandle, ClaimResources>,
											public HardwareResourceManager<KUKACartesianWrenchHandle, ClaimResources>,
											public HardwareResourceManager<JointHandle, ClaimResources>
	{
	public:
		/// these "using" directives are needed to disambiguate
		using HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::ResourceManager<KUKACartesianStiffnessHandle>::registerHandle;
		using HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::ResourceManager<KUKACartesianDampingHandle>::registerHandle;
		using HardwareResourceManager<KUKACartesianPoseHandle, ClaimResources>::ResourceManager<KUKACartesianPoseHandle>::registerHandle;
		using HardwareResourceManager<KUKACartesianWrenchHandle, ClaimResources>::ResourceManager<KUKACartesianWrenchHandle>::registerHandle;
		using HardwareResourceManager<JointHandle, ClaimResources>::ResourceManager<JointHandle>::registerHandle;
		
		
		typedef JointHandle  ResourceHandleType;
		
		/// getHandle needs to be discriminated as there is no way of deducing which functions to call (only differ based on return type)
		/// unless using a Proxy class, and exploiting the cast operator
		class handleProxy
		{
			KUKACartesianCommandInterface* myOwner;
			const std::string& myName;
			public:
				handleProxy( KUKACartesianCommandInterface* owner, const std::string& name ) : myOwner(owner), myName(name) {}
				/// the commented implementation is more generic, and more error prone: could try to call also different cast,
				/// and may thus result in inconsistencies (as HardwareResourceManager<T,ClaimResources> only works for some T's
				// template<class T>
				// operator T() const
				// {
				//     return myOwner->HardwareResourceManager<T, ClaimResources>::getHandle(myName);
				// }
				operator KUKACartesianStiffnessHandle() const
				{
					return myOwner->HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::getHandle(myName);
				}
				operator KUKACartesianDampingHandle() const
				{
					return myOwner->HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::getHandle(myName);
				}
				operator KUKACartesianPoseHandle() const
				{
					return myOwner->HardwareResourceManager<KUKACartesianPoseHandle, ClaimResources>::getHandle(myName);
				}
				operator KUKACartesianWrenchHandle() const
				{
					return myOwner->HardwareResourceManager<KUKACartesianWrenchHandle, ClaimResources>::getHandle(myName);
				}
				operator JointHandle() const
				{
					return myOwner->HardwareResourceManager<JointHandle, ClaimResources>::getHandle(myName);
				}
		};
		
		handleProxy getHandle(const std::string& name)
		{
			return handleProxy(this, name);
		}
		
		/// get names for all resources
		std::vector<std::string> getNames() const
		{
			std::vector<std::string> vect_joint_names = this->HardwareResourceManager<JointHandle, ClaimResources>::getNames();
			std::vector<std::string> vect_cart_siff_names = this->HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::getNames();
			std::vector<std::string> vect_cart_damp_names = this->HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::getNames();
			std::vector<std::string> vect_cart_pose_names = this->HardwareResourceManager<KUKACartesianPoseHandle, ClaimResources>::getNames();
			std::vector<std::string> vect_cart_wrench_names = this->HardwareResourceManager<KUKACartesianWrenchHandle, ClaimResources>::getNames();
			
			
			vect_joint_names.insert(vect_joint_names.end(), std::make_move_iterator(vect_cart_siff_names.begin()), std::make_move_iterator(vect_cart_siff_names.end()));
			vect_joint_names.insert(vect_joint_names.end(), std::make_move_iterator(vect_cart_damp_names.begin()), std::make_move_iterator(vect_cart_damp_names.end()));
			vect_joint_names.insert(vect_joint_names.end(), std::make_move_iterator(vect_cart_pose_names.begin()), std::make_move_iterator(vect_cart_pose_names.end()));
			vect_joint_names.insert(vect_joint_names.end(), std::make_move_iterator(vect_cart_wrench_names.begin()), std::make_move_iterator(vect_cart_wrench_names.end()));
			
			return vect_joint_names;
		}
		
		/// Clear the resources this interface is claiming
		void clearClaims()
		{
			this->HardwareResourceManager<JointHandle, ClaimResources>::clearClaims();
			this->HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::clearClaims();
			this->HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::clearClaims();
			this->HardwareResourceManager<KUKACartesianPoseHandle, ClaimResources>::clearClaims();
			this->HardwareResourceManager<KUKACartesianWrenchHandle, ClaimResources>::clearClaims();
			
			return;
		}
		
		/// Get the list of resources this interface is currently claiming
		std::set<std::string> getClaims() const
		{
			std::set<std::string> set_joint_claims = this->HardwareResourceManager<JointHandle, ClaimResources>::getClaims();
			std::set<std::string> set_stiff_claims = this->HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::getClaims();
			std::set<std::string> set_damp_claims = this->HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::getClaims();
			std::set<std::string> set_pose_claims = this->HardwareResourceManager<KUKACartesianPoseHandle, ClaimResources>::getClaims();
			std::set<std::string> set_wrench_claims = this->HardwareResourceManager<KUKACartesianWrenchHandle, ClaimResources>::getClaims();
			
			set_joint_claims.insert(set_stiff_claims.begin(), set_stiff_claims.end());
			set_joint_claims.insert(set_damp_claims.begin(), set_damp_claims.end());
			set_joint_claims.insert(set_pose_claims.begin(), set_pose_claims.end());
			set_joint_claims.insert(set_wrench_claims.begin(), set_wrench_claims.end());
			
			return set_joint_claims;
		}
	};
	
	/// \ref KUKACartesianInterface for commanding cartesian-based KUKA LWR 4+.
	class KUKACartesianInterface : public KUKACartesianCommandInterface {};

}

#endif
