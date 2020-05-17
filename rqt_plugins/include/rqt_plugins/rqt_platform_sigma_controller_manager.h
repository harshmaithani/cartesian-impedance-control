/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#ifndef controller_manager_plugin_H
#define controller_manager_plugin_H

#include "ros/ros.h"

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QTreeWidget>
#include <QMap>

#define TRACE_ControllerManagerPlugin_ACTIVATED 0

namespace platform_sigma_plugins_ns {
	
	class ControllerManagerPlugin : public rqt_gui_cpp::Plugin
	{
		Q_OBJECT
		
		enum ActionController { START, STOP };
			
		public:

			ControllerManagerPlugin();

			virtual void initPlugin(qt_gui_cpp::PluginContext& context);

			virtual void shutdownPlugin();

			virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
									  qt_gui_cpp::Settings& instance_settings) const;

			virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
										 const qt_gui_cpp::Settings& instance_settings);
		 public slots:
		 
			void tree_controllers_widget_ContextMenu(const QPoint& aPoint);
			void ns_combo_changed(int);
		 
		 private:
			QWidget* widget_;
			QVBoxLayout* vlayout_outer_;
			QHBoxLayout* hlayout_top_;
			
			QLabel* ns_label_;
			QComboBox* ns_combo_;
			QTreeWidget* tree_controllers_widget_;
			
			QStringList column_names_list_, column_names_pretty_list_;
			QMap<QString, ros::ServiceClient> map_list_service_client_;
			QMap<QString, ros::ServiceClient> map_switch_service_client_;
			
			
			void setupROSComponents_();
			void shutdownROSComponents_();
			
			void updateListControllers_();
			void switchController_(QString & name, ActionController action);
		
	}; // End of class

} // End of namespace

#endif
