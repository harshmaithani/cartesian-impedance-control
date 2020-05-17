/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include "rqt_plugins/rqt_platform_sigma_controller_manager.h"
#include <pluginlib/class_list_macros.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
// cf /opt/ros/indigo/include/controller_manager_msgs

#include <QMenu>

namespace platform_sigma_plugins_ns {
	
	ControllerManagerPlugin::ControllerManagerPlugin()
	: rqt_gui_cpp::Plugin(), widget_(0),vlayout_outer_(0),hlayout_top_(0),ns_label_(0),ns_combo_(0),tree_controllers_widget_(0)
	{
		setObjectName("Plugin Controller Manager");
	}

	void ControllerManagerPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		setupROSComponents_();
		
		// create a main widget named widget_
		widget_ = new QWidget();
		widget_->setWindowTitle("Controller Manager");
		
		// create layouts
        vlayout_outer_ = new QVBoxLayout();
        vlayout_outer_->setObjectName("vertical_layout_outer");
        
        hlayout_top_ = new QHBoxLayout();
        hlayout_top_->setObjectName("horizontal_layout_top");
        
		vlayout_outer_->addLayout(hlayout_top_);
		
		// create content of horizontal layout
		// create a label
		ns_label_ = new QLabel();
        ns_label_->setObjectName("ns_label_");
        ns_label_->setText("Kuka Namespace :");
        QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        ns_label_->setSizePolicy(fixed_policy);
        hlayout_top_->addWidget(ns_label_);
        
        // create a combo box for kuka namespaces
        ns_combo_ = new QComboBox();
        ns_combo_->setObjectName("ns_combo_");
        ns_combo_->addItem("kuka_lwr_left");
        ns_combo_->addItem("kuka_lwr_right");
        
        connect(ns_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(ns_combo_changed(int)));
        
		hlayout_top_->addWidget(ns_combo_);
		
		// create tree/list widget of kuka controllers
		tree_controllers_widget_ = new QTreeWidget();
		tree_controllers_widget_ ->setObjectName("tree_controllers_widget_");
		
		column_names_list_ << "name" << "state" << "type" << "hw_iface" << "resources";
		column_names_pretty_list_ << "Controller Name" << "State" << "Type" << "HW Interface" << "Claimed Resources";
		
		tree_controllers_widget_->setColumnCount(column_names_list_.size());
		tree_controllers_widget_->setHeaderLabels(column_names_pretty_list_);
		tree_controllers_widget_->sortByColumn(0, Qt::AscendingOrder);
		tree_controllers_widget_->setContextMenuPolicy(Qt::CustomContextMenu);
		
		connect(tree_controllers_widget_, SIGNAL( customContextMenuRequested( const QPoint& ) ),
             this, SLOT( tree_controllers_widget_ContextMenu( const QPoint& ) ) );
             
		vlayout_outer_->addWidget(tree_controllers_widget_);
		
		// set widget_ to main widget
		widget_->setLayout(vlayout_outer_);
		context.addWidget(widget_);
		
		updateListControllers_();
	}
	
	void ControllerManagerPlugin::tree_controllers_widget_ContextMenu(const QPoint& aPoint)
	{
		QTreeWidgetItem* itemSelected = tree_controllers_widget_->itemAt(aPoint);
		QString state = itemSelected->text(1);
		QString name = itemSelected->text(0);
		
		#if TRACE_ControllerManagerPlugin_ACTIVATED
			ROS_INFO("name = %s, state = %s",name.toStdString().c_str(), state.toStdString().c_str());
		#endif
		
		QMenu menu(tree_controllers_widget_);
		QIcon icon_start(QString::fromUtf8(":/icons/media-playback-start.png"));
		QIcon icon_stop(QString::fromUtf8(":/icons/media-playback-stop.png"));
		QAction* action_start;
		QAction* action_stop;
		
		if (state == "running")
		{
			action_stop = menu.addAction(icon_stop,"Stop Controller");
		}
		else
		if (state == "stopped")
		{
			action_start = menu.addAction(icon_start,"Start Controller");
		}
		
		QAction* action_menu_selected = menu.exec(tree_controllers_widget_->mapToGlobal(aPoint));
		
		if (action_menu_selected != NULL)
		{
		
			if (action_menu_selected->text() == action_start->text())
			{
				#if TRACE_ControllerManagerPlugin_ACTIVATED
					ROS_INFO("action_start !");
				#endif
				switchController_(name, ActionController::START);
			}
			else
				if (action_menu_selected->text() == action_stop->text())
				{
					#if TRACE_ControllerManagerPlugin_ACTIVATED
						ROS_INFO("action_stop !");
					#endif
					switchController_(name, ActionController::STOP);
				}
		}
		
	}
	
	
	void ControllerManagerPlugin::switchController_(QString & name, ActionController action)
	{
		ros::ServiceClient switch_client = map_switch_service_client_[ns_combo_->currentText()];
		
		controller_manager_msgs::SwitchController switch_controller;
		switch_controller.request.start_controllers.clear();
		switch_controller.request.stop_controllers.clear();
		
		if (action == ActionController::START)
		{
			#if TRACE_ControllerManagerPlugin_ACTIVATED
				ROS_INFO("switch controller Start !");
			#endif
			switch_controller.request.start_controllers.push_back(name.toStdString().c_str());
		}
		else
			if (action == ActionController::STOP)
			{
				#if TRACE_ControllerManagerPlugin_ACTIVATED
					ROS_INFO("switch controller Stop !");
				#endif
				switch_controller.request.stop_controllers.push_back(name.toStdString().c_str());
			}
		switch_controller.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;	
		switch_client.call(switch_controller);
		
		updateListControllers_();
    }
	
	void ControllerManagerPlugin::ns_combo_changed(int index)
	{
		updateListControllers_();
	}

	void ControllerManagerPlugin::shutdownPlugin()
	{
		shutdownROSComponents_();
		
		disconnect(ns_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(ns_combo_changed(int)));
		
		disconnect(tree_controllers_widget_, SIGNAL( customContextMenuRequested( const QPoint& ) ),
             this, SLOT( tree_controllers_widget_ContextMenu( const QPoint& ) ) );
             
		vlayout_outer_->removeWidget(tree_controllers_widget_);
		hlayout_top_->removeWidget(ns_combo_);
		hlayout_top_->removeWidget(ns_label_);
		
		if (ns_label_)
			delete ns_label_;
		
		if (ns_combo_)
			delete ns_combo_;
		
		if (tree_controllers_widget_)
			delete tree_controllers_widget_;
			
		if (hlayout_top_)
			delete hlayout_top_;
		 
		if (vlayout_outer_)
			delete vlayout_outer_;
		
	}

	void ControllerManagerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
								qt_gui_cpp::Settings& instance_settings) const
	{
		// TODO save intrinsic configuration, usually using:
		// instance_settings.setValue(k, v)
	}

	void ControllerManagerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
								   const qt_gui_cpp::Settings& instance_settings)
	{
		// TODO restore intrinsic configuration, usually using:
		// v = instance_settings.value(k)
	}
	
	void ControllerManagerPlugin::setupROSComponents_()
	{
		map_list_service_client_.insert("kuka_lwr_left",getNodeHandle().serviceClient<controller_manager_msgs::ListControllers>("/kuka_lwr_left/controller_manager/list_controllers")); 
		map_list_service_client_.insert("kuka_lwr_right",getNodeHandle().serviceClient<controller_manager_msgs::ListControllers>("/kuka_lwr_right/controller_manager/list_controllers")); 
	
		map_switch_service_client_.insert("kuka_lwr_left",getNodeHandle().serviceClient<controller_manager_msgs::SwitchController>("/kuka_lwr_left/controller_manager/switch_controller")); 
		map_switch_service_client_.insert("kuka_lwr_right",getNodeHandle().serviceClient<controller_manager_msgs::SwitchController>("/kuka_lwr_right/controller_manager/switch_controller"));	
	}
	
	void ControllerManagerPlugin::shutdownROSComponents_()
	{
		map_list_service_client_["kuka_lwr_left"].shutdown();
		map_list_service_client_["kuka_lwr_right"].shutdown();
		
		map_switch_service_client_["kuka_lwr_left"].shutdown();
		map_switch_service_client_["kuka_lwr_right"].shutdown();
	}
	
	void ControllerManagerPlugin::updateListControllers_()
	{
		#if TRACE_ControllerManagerPlugin_ACTIVATED
			ROS_INFO("ControllerManagerPlugin::updateListControllers_() !");
		#endif
		
		ros::ServiceClient controller_list_client = map_list_service_client_[ns_combo_->currentText()];

		controller_manager_msgs::ListControllers controller_list;
	
		controller_list_client.call(controller_list);
		QString ressources;
		QStringList strListRessources;
		
		tree_controllers_widget_->clear();
	
		for (unsigned int i=0;i<controller_list.response.controller.size() ;i++ )
		{
			// Create a new item
			QTreeWidgetItem* new_item = new QTreeWidgetItem(tree_controllers_widget_);
			new_item->setText(0, controller_list.response.controller[i].name.c_str());
			new_item->setText(1, controller_list.response.controller[i].state.c_str());
			new_item->setText(2, controller_list.response.controller[i].type.c_str());
			new_item->setText(3, controller_list.response.controller[i].hardware_interface.c_str());
		   
			
			strListRessources.clear();
			for (unsigned int j=0;j<controller_list.response.controller[i].resources.size() ;j++ )
			{
				strListRessources << controller_list.response.controller[i].resources[j].c_str();
			}
			
			new_item->setText(4, strListRessources.join(","));
				
		}
	}

} // End of namespace

PLUGINLIB_DECLARE_CLASS(platform_sigma_plugins_ns, ControllerManagerPlugin, platform_sigma_plugins_ns::ControllerManagerPlugin, rqt_gui_cpp::Plugin)

