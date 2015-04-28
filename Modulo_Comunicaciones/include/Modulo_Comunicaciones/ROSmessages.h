/**
 * @file   ROSmessages.h
 * @brief  Fichero de cabecera con la inclusión de los mensajes ROS
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 * @addtogroup CommVehicle
 * @{
 */



#ifndef ROSMESSAGES_H
#define	ROSMESSAGES_H

#include <Modulo_Comunicaciones/constantCommunication.h>

Common_files::msg_cameraPtr mens_cam(new Common_files::msg_camera);
Common_files::msg_gpsPtr mens_gps(new Common_files::msg_gps);
Common_files::msg_errorPtr mens_error(new Common_files::msg_error);
Common_files::msg_modePtr mens_mode(new Common_files::msg_mode);
Common_files::msg_waypointPtr mens_waypoint(new Common_files::msg_waypoint);
Common_files::msg_com_teleopPtr mens_teleop(new Common_files::msg_com_teleop);
Common_files::msg_backupPtr mens_backup(new Common_files::msg_backup);
Common_files::msg_availablePtr mens_available(new Common_files::msg_available);
Common_files::msg_streamPtr mens_file(new Common_files::msg_stream);
Common_files::msg_fcn_auxPtr mens_fcn_aux(new Common_files::msg_fcn_aux); 
Common_files::msg_info_stopPtr mens_info_stop(new Common_files::msg_info_stop);
Common_files::msg_ctrl_cameraPtr mens_ctrl_cam(new Common_files::msg_ctrl_camera);

#endif	/* ROSMESSAGES_H */

/**
 *@}
 */