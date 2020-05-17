/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#ifndef joint_position_plugin_H
#define joint_position_plugin_H

// rqt
#include <rqt_gui_cpp/plugin.h>

// Qt graphics
/*#include <QtGui/QWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QLineEdit>
#include <QtGui/QComboBox>
#include <QtGui/QTabWidget>
#include <QtGui/QTableWidget>

// Qt core
#include <QtCore/QTimer>*/

// Qt graphics
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QLineEdit>
#include <QComboBox>
#include <QTabWidget>
#include <QTableWidget>

// Qt core
#include <QTimer>


// Qwt graphics
#include <qwt_slider.h>
#include <qwt_text_label.h>

#include <qwt_plot_grid.h>
#include <qwt_legend.h>
#include <qwt_symbol.h>

#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>

// ROS msgs
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

// ROS
#include "ros/ros.h"

// Curves widget
#include "qtplotchecked.h"

// Joint Position Sliders widget
#include "qtpositionsliders.h"

// Velocity sliders widget
#include "qtvelocitysliders.h"

#define TRACE_JointPositionPlugin_ACTIVATED 1

namespace platform_sigma_plugins_ns {
	
	class JointPositionPlugin : public rqt_gui_cpp::Plugin
	{
		Q_OBJECT
		
		public:

			JointPositionPlugin();

			virtual void initPlugin(qt_gui_cpp::PluginContext& context);

			virtual void shutdownPlugin();

			virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
									  qt_gui_cpp::Settings& instance_settings) const;

			virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
										 const qt_gui_cpp::Settings& instance_settings);
		 public slots:
		 
			void setValueLineJ0(double value);
			void setValueLineJ1(double value);
			void setValueLineJ2(double value);
			void setValueLineJ3(double value);
			void setValueLineJ4(double value);
			void setValueLineJ5(double value);
			void setValueLineJ6(double value);
			
			void updateValueSliderJ0();
			void updateValueSliderJ1();
			void updateValueSliderJ2();
			void updateValueSliderJ3();
			void updateValueSliderJ4();
			void updateValueSliderJ5();
			void updateValueSliderJ6();
			
			void sendPosition();
			void sendMaxVelocity();
			
			void doUpdateLabelJs(QVector<double> positions);
			
			void ns_combo_changed(int);
			void resetSlidersPositions();
			void resetSlidersVelocity();
			
			void doUpdateCurves();
			
		  signals:
				
			void updateLabelJs(QVector<double> positions);
			void updateCurves();
		 
		  private:
		  
			QTabWidget* tab_widget_;
			
			QWidget* widget_positions_, * widget_velocities_, * widget_global_;
			
			QVBoxLayout* vlayout_global_, * vlayout_positions_, * vlayout_velocities_;
			
			QPushButton* button_send_positions_, *button_send_max_velocity_;
			
			QComboBox* ns_combo_;
			
			/* Publishers && Subscribers */
			QMap<QString, ros::Publisher> map_pub_joint_position_;
			QMap<QString, ros::Publisher> map_pub_joint_velocity_;
			QMap<QString, ros::Subscriber> map_sub_joint_handle_;
			QMap<QString, ros::ServiceClient> map_get_velocity_service_client_;
			
			QMap<QString, QVector<double> > map_selected_joint_values_;
			QMap<QString, QVector<double> > map_selected_joint_velocities_;
			
			QMap<QString, QVector<double> > map_current_joint_state_values_;
			QMap<QString, bool > map_sliders_is_init_;
			
			QVector<double> vect_zero_position_;
			
			/* Ros msg */
			std_msgs::Float64MultiArray joint_position_msg_, joint_velocity_msg_;
			
			void setupROSComponents_();
			void shutdownROSComponents_();
			
			
			void jsCallback_left_(const sensor_msgs::JointState::ConstPtr& msg);
			void jsCallback_right_(const sensor_msgs::JointState::ConstPtr& msg);
			
			platform_sigma_plugins_ns::QtPlotChecked *plot_checked_;
			platform_sigma_plugins_ns::QtPositionSliders *position_sliders_;
			platform_sigma_plugins_ns::QtVelocitySliders *velocity_sliders_;
			
			QTimer * timer_;
			
			double firstTime_;
	}; // End of class
	
} // End of namespace

#endif
